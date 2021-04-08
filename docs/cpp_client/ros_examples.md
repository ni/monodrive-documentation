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

<!-- ## Example Description -->
<!-- example is out of date.  -->
