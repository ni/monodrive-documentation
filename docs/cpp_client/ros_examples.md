# ROS Examples

## monoDrive ROS Client Examples

The monoDrive C++ Client comes with a simple example to connect the ROS client
to a running instance of the monoDrive Simulator or Scenario Editor and 
automatically steer the ego vehicle for lane keeping. The example can be found
in the `monodrive-client/cpp-client/ros-examples/` directory. 

### Building the Example

The example requires `catkin_make` to build which is available from the ROS 
distribution setup during installation. To build:

    $ cd cpp-client/ros-examples
    $ catkin_make

### Launching the example

To launch the monoDrive ROS example, open a terminal and create 3 tabs in the 
`cpp-client/ros-examples` directory:

1. In one tab, launch `rosbridge`: 

        $ roscore

2. To start the vehicle control node:

        $ rosrun vehicle_control node

    **Note:** The vehicle_control example only requires the monodrive_msgs package 
    and provides an example of how to connect your code to monoDrive through ROS 
    messages.

3. The next command requires that the monoDrive Simulator is running. To create the simulator node:

        $ rosrun simulator_control node

### ROS Sensor Message Types

The following sensor message types are supported:

| Sensor | ROS Message Type |
| ------------ | ---------- | 
| Camera and Semantic Camera | sensor_msgs/Image |
| IMU | sensor_msgs/Imu |
| Lidar | sensor_msgs/PointCloud2 |
| State Sensor| monodrive_msgs/StateSensor |
| Waypoint Sensor| monodrive_msgs/WaypointSensor |

### Message Conversion
monoDrive provides a message conversion factory class for converting monoDrive frame data to/from ROS messages. This class is defined in `monodrive/ros/src/monodrive_msgs/include/MessageFactory.h`. An example usage:

```cpp
auto& data = *static_cast<RadarFrame*>(frame); // parsed monoDrive radar sensor frame
monodrive_msgs::Radar msg = monodrive_msgs::MessageFactory::FromMonoDriveFrame(data); // convert to ROS message
pub_radar.publish(msg); // publish to ROS topic
```

## Example Description

To create a vehicle control message for publishing to the simulator:

```cpp
// create vehicle controller publisher and sensor subscriber
node_handle = std::make_shared<ros::NodeHandle>(ros::NodeHandle());
vehicle_control_pub = node_handle->advertise<monodrive_msgs::VehicleControl>(
    "/monodrive/vehicle_control", 1);
```

To subscribe to simulator state sensor messages for vehicle feedback:
```cpp
state_sensor_sub = node_handle->subscribe(
    "/monodrive/state_sensor", 1, &state_sensor_callback)
```

The state sensor call back can be as simple as:

```cpp
void state_sensor_callback(const monodrive_msgs::StateSensor &state_sensor_msg){
    state_data = state_sensor_msg;
}
```

These examples query the simulator for the OpenDrive Map definition, parse it using the client's map API, and query the resulting data structure to determine the target location for the ego vehicle. The map query to the simulator is sent and read here:

```cpp
// Get the current map data from and parse it
MapConfig map_request;
map_request.format = "opendrive";
ApiMessage response;
sim0.sendCommand(map_request.message(), &response);
map_data = carla::opendrive::OpenDriveParser::Load(nlohmann::json::parse(
    response.get_message().get<std::string>()).at("map")
);
if(!map_data.has_value()) {
    std::cerr << "ERROR! Unable to get map data from simulator!" << std::endl;
    return -1;
}
```

To issue vehicle control commands for keeping the ego vehicle within its current 
lane, first grab the vehicle information from the state sensor

```cpp
void control_vehicle(){
    monodrive_msgs::VehicleState vs;
    for(auto& vehicle : state_data.vehicles){
        if(vehicle.name == "Ego"){
            vs = vehicle;
        }
    }
    Eigen::VectorXd position(3);
    position << vs.odometry.pose.pose.position.x,
        vs.odometry.pose.pose.position.y,
        vs.odometry.pose.pose.position.z;
    Eigen::Quaternion<double> orientation(
        vs.odometry.pose.pose.orientation.w,
        vs.odometry.pose.pose.orientation.x,
        vs.odometry.pose.pose.orientation.y,
        vs.odometry.pose.pose.orientation.z
    );
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
    } else {
    std::cerr << "WARNING! Unable to compute NEXT waypoint!" << std::endl;
    }
} else {
    std::cerr << "WARNING! Unable to compute CURRENT waypoint!" << std::endl;
}

forwardVector << 1, 0, 0;
forwardVector = orientation * forwardVector;
Eigen::VectorXd dirToNextPoint = nextPoint - position;
dirToNextPoint.normalize();

double angle = -dirToNextPoint.head<3>().cross(forwardVector.head<3>())[2];
```

Create the new control command to the vehicle and send it:

```cpp
monodrive_msgs::VehicleControl msg;
msg.name = "Ego";
msg.throttle = 0.75f;
msg.brake = 0.f;
msg.steer = angle;
msg.drive_mode = 1;

vehicle_control_pub.publish(msg);
```

The command generated by the above function is send to the simulator in the 
main loop:

```cpp
    while(ros::ok()){
        control_vehicle();
        ros::spinOnce();
        rate.sleep();
    }
```