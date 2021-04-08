# ROS Client Quick Start

## Guide

The monoDrive simulator integrates with ROS by using TCP/IP streams to receive sensor data, converting that data on a frame by frame basis to ROS messages, and publishing those on the desired ROS topic. To support this pattern, there is a message conversion utility class in `monodrive/ros/src/monodrive_msgs/include/MessageFactory.h` that facilitates converting raw sensor frame data to ROS messages and vice-versa.

## ROS Ubuntu Prerequisites

- [monoDrive C++ Client](https://github.com/monoDriveIO/monodrive-client#monodrive-c-client)
- ROS 
    - [Ubuntu 18.04 Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) 
    - [Ubuntu 20.04 Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

- ROS Bridge:

    - _Ubuntu 18.04_
        <pre><code class="language-bash hljs">sudo apt-get install ros-melodic-rosbridge-suite</code></pre>

    - _Ubuntu 20.04_
        <pre><code class="language-bash hljs">sudo apt-get install ros-noetic-rosbridge-suite</code></pre>

- ROS Joy (required for steering wheel example):

    - _Ubuntu 18.04_
        <pre><code class="language-bash hljs">sudo apt-get install ros-melodic-joy</code></pre>

    - _Ubuntu 20.04_
        <pre><code class="language-bash hljs">sudo apt-get install ros-noetic-joy</code></pre>

## Build monoDrive ROS Packages and examples

1. Execute the following to build the monoDrive ROS library:
    <pre><code class="language-bash hljs">cd ./monodrive/ros 
catkin_make
source devel/setup.bash</code></pre>

2. Execute the following to build the ROS packages:
    <pre><code class="language-bash hljs">cd ./examples/ros
catkin_make
source devel/setup.bash</code></pre>

## Running the ROS examples

To launch the monoDrive example, create 3 tabs/windows and run each command in a separate terminal:

1. Launch rosbridge, you can leave this running:
    <pre><code class="language-bash hljs">roscore</code></pre>

2. Make sure the monoDrive simulator is running. Launch the simulator control node to configure the scenario
and begin forwarding sensor and control messages.
    <pre><code class="language-bash hljs">rosrun simulator_control node</code></pre>

3. Start the vehicle control node which will subscribe to the state sensor topic and publish vehicle controls.
    <pre><code class="language-bash hljs">rosrun vehicle_control node</code></pre>
   
## Using the G920 Logitech wheel

### Configuring the Joystick
1. Connect your joystick to your computer. Check if Linux recognized your joystick.
    <pre><code class="language-bash hljs">ls /dev/input/</code></pre>
    
    From the list verify that you can see `jx0` or `jsX` in the list, this will mean Linux recognize your joystick.

2. Make the joystick accessible to the ROS joy node. Where `jsX` is `js0` from the example above.
    <pre><code class="language-bash hljs">sudo chmod a+rw /dev/input/jsX</code></pre>

3. Setup your the joystick device to use on the joy node . Assumig your joystick is `js0`
    <pre><code class="language-bash hljs">rosparam set joy_node/dev "/dev/input/js0"</code></pre>

4. *Optional: You can also specify the dead zone as a parameter*
    <pre><code class="language-bash hljs">rosparam set joy_node/deadzone 0.05</code></pre>

### Launching the example

To launch the monoDrive examples create 4 tabs and run each command in a separate terminal:   

1. Launch rosbridge, you can leave this running:
    <pre><code class="language-bash hljs">roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True</code></pre>

1. Start the joy_node.
    <pre><code class="language-bash hljs">rosrun joy joy_node</code></pre>

1. Start the wheel vehicle control node which will subscribe to the state sensor topic and publish vehicle controls (the simulator does not need to be running)
    <pre><code class="language-bash hljs">rosrun wheel_vehicle_control node</code></pre>

    **Note**: The vehicle_control example only requires the monodrive_msgs package and provides an example of how to connect your code to monoDrive through ROS messages.

1. Make sure the monoDrive simulator is running since the next command will connect to and start the simulator scenario running.
     <pre><code class="language-bash hljs">rosrun simulator_control node</code></pre>

    **Note:** The following table show how the buttons in the G920 wheel map to the ROS message.   


<h4 style="text-align:center;"> Axes </h4>

| Function | ROS message |
|----------|-------------|
| Steering |joy->axes[0] |
| Throttle |joy->axes[1] |
| Brake | joy->axes[2]|
| Clutch | joy->axes[3]|
| Directional pad RIGHT | joy->axes[4]|
| Directional pad LEFT | joy->axes[4]|
| Directional pad UP| joy->axes[5]|
| Directional pad DOWN  | joy->axes[5]|

<h4 style="text-align:center;"> Buttons </h4>

| Button   |      ROS message      |
|----------|:-------------:|
| A |joy->buttons[0] |
| B |joy->buttons[1] |
| X | joy->buttons[2]|
| Y | joy->buttons[3]|
| RB | joy->buttons[4]|
| LB | joy->buttons[5]|
| Menu button| joy->buttons[6]|
| View button  | joy->buttons[7]|
| RSB | joy->buttons[8]|
| LSB | joy->buttons[9]|
| Xbox button | joy->buttons[10]|   

