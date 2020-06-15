# C++ Examples

## monoDrive C++ Client Examples

The monoDrive C++ Client comes with examples for connecting to the monoDrive 
Simulator or Scenario Editor and controlling the ego vehicle in both Replay and 
Closed Loop modes. The examples can be found in the `cpp-client/cpp-examples` 
directory in the `monodive-client` repo.

### Connecting to the Simulator

To connect to a running instance of the monoDrive Simulator or Scenario Editor 
in "Play" mode, read in configuration values and create an instance of a 
`Simulator` object:

```cpp
//Read JSON files in cpp_client/config directory
Configuration config(
    "simulator-cpp-client/config/simulator.json",
    "simulator-cpp-client/config/vehicle.json",
    "simulator-cpp-client/config/weather.json",
    "simulator-cpp-client/config/scenario.json"
);
// Default simulator port is 8999
Simulator& simulator = Simulator::getInstance(config, "127.0.0.1", 8999);
```

### Creating Sensors

Each sensor has a configuration `struct` that must be setup either by reading in 
a JSON configuration file or assigning values to the `struct` directly. Here, 
an example of constructing a camera sensor from JSON should be:

```cpp
// Read the JSON from file
std::ifstream in("/path/to/my/config.json");
nlohmann::json file;
file << in;

// Import into the camera config struct
CameraConfig camera_config;
camera_config.from_json(file);
```

Alternatively, the `struct` can be setup directly in the code:

```cpp
CameraConfig camera_config;
camera_config.server_ip = ip;
camera_config.listen_port = 8100;
camera_config.location.z = 200;
camera_config.rotation.pitch = -5;
camera_config.resolution = CameraConfig::Resolution(1024,1024);
```

After the sensor's configuration is created, all that needs to be done is 
configure the sensor:

```cpp
Sensor camera(CameraConfig);
camera.configure();
```

### Stepping in Replay Mode

After the `simulator` object is connected and the sensors are configured, the 
simulation can be stepped in replay mode:

```cpp
for(; idx < nSteps; idx++)
{	
    //step simulator
    task = std::async([&simulator, &idx](){
        return simulator.step(idx, 1);
    });
```

After each step, a sensor sample can be acquired using the sensor's `sample()` 
function:

```cpp
    //sample all sensors
    for(auto& sensor : sensors)
    {
        sensor.sample();
    }
    if(!task.get()){
        break;
    }
} // for
```

### Controlling the ego vehicle

As an alternative to Replay mode, a closed loop control over the ego vehicle can 
be done by sending control commands to the simulator, here the vehicle is 
requested to move forward with 50% throttle:

```cpp
simulator.send_command(ApiMessage(123, EgoControl_ID, true, 
    {   {"forward_amount", 0.5}, 
        {"right_amount", 0.0},
        {"brake_amount", 0.0},
        {"drive_mode", 1}
    }));
```

## Replay Example

A full example of controlling the ego vehicle in Replay mode can be found in 
`monodrive-client/cpp-client/cpp-examples/replay/replay_example.cpp`. This 
example demonstrates:

* Configuring and connecting to a running instance of the monoDrive Simulator
* Configuring and connecting to a Camera sensor (as discussed above)
* Configuring the Viewport Sensor for the camera on the simulator
* Stepping a monoDrive Trajectory File in Repaly mode (as discussed above)

The Viewport Sensor in this example controls where the camera view will be 
placed in the monoDrive Simulator or Scenario Editor. This sensor is not 
configured and sent to the `sim0` instance without needing to be sampled:

```cpp
ViewportCameraConfig vp_config;
vp_config.server_ip = ip;
vp_config.location.z = 200;
Sensor(vp_config).configure();
```

## Lane Follower Example

An example of controlling the ego vehicle in a closed loop mode can be found in 
the Lane Follower example provided in 
`monodrive-client/cpp-client/cpp-examples/replay/replay_example.cpp`. This 
example demonstrates:

* Configuring and connecting to a running instance of the monoDrive Simulator
* Configuring and connecting to a Camera sensor (as discussed above)
* Configuring the Viewport Sensor for the camera on the simulator (as discussed in the Replay Example)
* Configuring and connecting a State Sensor sensor to get state information from the ego vehicle
* Issuing control commands to the simulator to keep the ego vehicle within a lane

In this example, the State Sensor information is used to stream ego vehicle 
state information back to the client:

```cpp
StateConfig state_config;
state_config.desired_tags = {"vehicle", "ego"};
state_config.server_ip = ip;
state_config.listen_port = 8101;
state_config.debug_drawing = true;
state_config.undesired_tags = {""};
sensors.emplace_back(state_config);
```

This example uses the GeoJSON lanes from the monoDrive `Straightaway5k` map in 
order to calculate the distance of the ego vehicle to the current lane. The 
lanes can be read in as follows:

```cpp
lanespline = LaneSpline("cpp-examples/lane_follower/Straightaway5k.json");
```

To issue vehicle control commands for keeping the ego vehicle within its current 
lane, first grab the vehicle information from the state sensor

```cpp
EgoControlConfig planning(DataFrame* dataFrame){
    auto& frame = *static_cast<StateFrame*>(dataFrame);
    std::cout << "sample, game, wall " << frame.sample_count << " " << frame.game_time << " " << frame.wall_time << std::endl;
    VehicleState* vehicle_frame = nullptr;
    for(auto& vehicle : frame.vehicles){
        for(auto& tag : vehicle.state.tags){
            if(tag == "ego"){
                vehicle_frame = &vehicle;
                break;
            }
        }
    }
    if(vehicle_frame == nullptr){
        std::cout << "No ego vehicle in frame." << std::endl;
        return EgoControlConfig();
    }
    Eigen::VectorXd position(3);
    position << vehicle_frame->state.odometry.pose.position.x,
        vehicle_frame->state.odometry.pose.position.y,
        vehicle_frame->state.odometry.pose.position.z;
    Eigen::Quaternion<double> orientation(
        vehicle_frame->state.odometry.pose.orientation.w,
        vehicle_frame->state.odometry.pose.orientation.x,
        vehicle_frame->state.odometry.pose.orientation.y,
        vehicle_frame->state.odometry.pose.orientation.z
    );
```

Now compute the vehicle's current distance from the lane and steer the vehicle 
towards the correct position:

```cpp
  auto nearestIndex = lanespline.GetNearestPoint("road_0", "lane_2", position);
    auto& lane_points = lanespline.spline_map["road_0"]["lane_2"];
    int nextPointIndex = nearestIndex;
    if(nearestIndex >= lane_points.size()-4){
        nextPointIndex = (int)lane_points.size() - 1;
    }
    else{
        nextPointIndex += 3;
    }
    Eigen::VectorXd forwardVector(3);
    forwardVector << 1, 0, 0;
    forwardVector = orientation * forwardVector;
    auto nextPoint = lane_points[nextPointIndex];
    Eigen::VectorXd dirToNextPoint = nextPoint - position;
    dirToNextPoint.normalize();
    double angle = -dirToNextPoint.head<3>().cross(forwardVector.head<3>())[2];
    EgoControlConfig egoControl;
    egoControl.forward_amount = 0.75;
    egoControl.brake_amount = 0.0;
    egoControl.drive_mode = 1;
    egoControl.right_amount = (float)angle;
    return egoControl;
```    

Finally, create the new control command to the vehicle and send it:

```cpp
    nlohmann::json msg;
    msg["forward_amount"] = 0.75f;
    msg["brake_amount"] = 0.0f;
    msg["drive_mode"] = 1;
    msg["right_amount"] = angle;

    simulator.send_command(ApiMessage(777, EgoControl_ID, true, msg));
}
```