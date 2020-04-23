# ROS Examples

## monoDrive ROS Client Examples

The monoDrive C++ Client comes with a simple example to connect the ROS client
to a running instance of the monoDrive Simulator or Scenario Editor and 
automatically steer the EGO vehicle for lane keeping. The example can be found
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

        $ roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True

2. To start the vehicle control node:

        $ rosrun vehicle_control node

    **Note:** The vehicle_control example only requires the monodrive_msgs package 
    and provides an example of how to connect your code to monoDrive through ROS 
    messages.

3. The next command requires that the monoDrive Simulator is running. To create the simulator node:

        $ rosrun simulator_control node

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

Now a simple function to control the vehicle can be used for maintaining the 
current lane:

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

    auto nearestIndex = lanespline.GetNearestPoint("road_0", "lane_2", position);
    auto& lane_points = lanespline.spline_map["road_0"]["lane_2"];
    int nextPointIndex = nearestIndex;
    if(nearestIndex >= lane_points.size()-4){
        nextPointIndex = lane_points.size()-1;
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

    monodrive_msgs::VehicleControl msg;
    msg.name = "Ego";
    msg.throttle = 0.75f;
    msg.brake = 0.f;
    msg.steer = angle;
    msg.drive_mode = 1;

    vehicle_control_pub.publish(msg);
}
```

Finally issuing the control commands in a loop in `main`:

```cpp
    while(ros::ok()){
        control_vehicle();
        ros::spinOnce();
        rate.sleep();
    }
```