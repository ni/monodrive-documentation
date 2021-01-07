# C++ Examples

## monoDrive C++ Client Examples

The monoDrive C++ Client comes with examples for connecting to the monoDrive 
Simulator or Scenario Editor and controlling the ego vehicle in both Replay and 
Closed Loop modes. The examples can be found in the `examples/cpp` 
directory in the `monodive-client` repo.

### Connecting to the Simulator

To connect to a running instance of the monoDrive Simulator or Scenario Editor 
in "Play" mode, read in configuration values and create an instance of a 
`Simulator` object:

```cpp
//Read JSON files in cpp_client/config directory
Configuration config(
    "examples/config/simulator.json",
    "examples/config/weather.json",
    "examples/config/scenario.json"
);
// Default simulator port is 8999
Simulator& sim0 = Simulator::getInstance(config, "127.0.0.1", 8999);

if (!sim0.configure())
{
    return -1;
}
```

### Creating Sensors

Each sensor has a configuration `struct` that must be setup either by reading in 
a JSON configuration file or by creating a configuration object and assigning the appropriate values. For instance, constructing a camera sensor should look like:

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
// define desired sensors
    std::vector<std::shared_ptr<Sensor>> sensors;

    CameraConfig fc_config;
    fc_config.server_ip = sim0.getServerIp();
    fc_config.server_port = sim0.getServerPort();
    fc_config.listen_port = 8100;
    fc_config.location.z = 225;
    fc_config.rotation.pitch = -5;
    fc_config.resolution = Resolution(1920,1080);
    fc_config.annotation.include_annotation = true;
    fc_config.annotation.desired_tags = {"traffic_sign"};
```

After the sensor's configuration is created, all that needs to be done is 
configure the sensor:

```cpp
sensors.push_back(std::make_shared<Sensor>(std::make_unique<CameraConfig>(fc_config)));
for (auto& sensor : sensors)
{
    sensor->configure();
}
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
`monodrive-client/examples/cpp/replay/replay.cpp`. This 
example demonstrates:

* Configuring and connecting to a running instance of the monoDrive Simulator
* Configuring and connecting to a Camera sensor (as discussed above)
* Configuring the Viewport Sensor for the camera on the simulator
* Stepping a monoDrive Trajectory File in Replay mode (as discussed above)

The Viewport Sensor in this example controls where the camera view will be 
placed in the monoDrive Simulator or Scenario Editor. This sensor is not 
configured and sent to the `sim0` instance without needing to be sampled:

```cpp
    ViewportCameraConfig vp_config;
    vp_config.server_ip = sim0.getServerIp();
    vp_config.server_port = sim0.getServerPort();
    vp_config.location.z = 200;
    vp_config.resolution = Resolution(256,256);
    Sensor(std::make_unique<ViewportCameraConfig>(vp_config)).configure();
```

## Lane Follower Examples

Two examples of controlling the ego vehicle in a closed loop mode can be found in 
the Lane Follower example provided in 
`monodrive-client/examples/cpp/lane_follower/`. These examples demonstrate:

* Configuring and connecting to a running instance of the monoDrive Simulator
* Configuring and connecting to a Camera sensor (as discussed above)
* Configuring the Viewport Sensor for the camera on the simulator (as discussed in the Replay Example)
* Configuring and connecting a State Sensor sensor to get state information from the ego vehicle
* Issuing control commands to the simulator to keep the ego vehicle within a lane

The State Sensor information is used to stream ego vehicle 
state information back to the client:

```cpp
StateConfig state_config;
state_config.desired_tags = {"ego"};
state_config.server_ip = sim0.getServerIp();
state_config.server_port = sim0.getServerPort();
state_config.listen_port = 8101;
state_config.debug_drawing = true;
state_config.undesired_tags = {""};
sensors.push_back(std::make_shared<Sensor>(std::make_unique<StateConfig>(state_config)));
```

These examples query the simulator for the OpenDrive Map definition, parse it using the client's map API, and query the resulting data structure to determine the target location for the ego vehicle. The map query to the simulator is sent and read here:
```cpp
    MapConfig map_request;
    map_request.format = "opendrive";
    ApiMessage response;
    sim0.sendCommand(map_request.message(), &response);
    map_data = carla::opendrive::OpenDriveParser::Load(
        nlohmann::json::parse(response.get_message().get<std::string>()).at("map")
    );
```

To issue vehicle control commands for keeping the ego vehicle within its current 
lane, first grab the vehicle information from the state sensor

```cpp
EgoControlConfig planning(DataFrame* dataFrame){
    auto& frame = *static_cast<StateFrame*>(dataFrame);

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
}
```

Now compute the vehicle's current distance from the lane and steer the vehicle 
towards the correct position:

```cpp
    auto currentWp = map_data->GetClosestWaypointOnRoad(carla::geom::Location(
    float(position[0] / 100.0), 
    float(position[1] / 100.0),
    float(position[2] / 100.0)));
    if(currentWp.has_value()) {
      auto nextWps = map_data->GetNext(*currentWp, 6.0);
      auto currentPos = map_data->ComputeTransform(*currentWp).location;
      if(nextWps.size() > 0) {
          auto nextLocation = map_data->ComputeTransform(nextWps[0]).location;
          nextPoint = Eigen::Vector3d(nextLocation.x, nextLocation.y, nextLocation.z)*100.0;
      } 
    } 

    Eigen::VectorXd dirToNextPoint = nextPoint - position;
    dirToNextPoint.normalize();

    double angle = -dirToNextPoint.head<3>().cross(forwardVector.head<3>())[2];

    // use PID speed controller for throttle
    Eigen::VectorXd velocity(3);
    velocity << vehicle_frame->state.odometry.linear_velocity.x,
        vehicle_frame->state.odometry.linear_velocity.y,
        vehicle_frame->state.odometry.linear_velocity.z;
    double speed = velocity.norm();

    double dt = last_time > 0 ? frame.game_time - last_time : 0.1;
    last_time = frame.game_time;

    float throttle = last_throttle;
    if (dt) {
        throttle = pid.pid(float(desired_speed - speed), float(dt));
        last_throttle = throttle;
    }

    // form controls response
    EgoControlConfig egoControl;
    egoControl.forward_amount = std::max(0.0f, throttle);
    egoControl.brake_amount = std::min(0.0f, throttle);
    egoControl.drive_mode = 1;
    egoControl.right_amount = (float)angle;

    return egoControl;
}
```    

Finally, create the new control command to the vehicle and send it:

```cpp
    sim0.sendCommand(ApiMessage(777, EgoControl_ID, true, egoControl.dump()));
}
```

The *Fixed Step Example*, `monodrive-client/examples/cpp/lane_follower/fixed_step.cpp`, contains a fixed amount of time between each step of the simulation physics in order to allow slow perception algorithms to keep up.

The *Real Time Example*, `monodrive-client/examples/cpp/lane_follower/real_time.cpp`, enables the simulation physics to go as fast as they can. 
