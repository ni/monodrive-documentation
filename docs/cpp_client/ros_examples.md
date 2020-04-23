# ROS Examples

## monoDrive ROS Client Examples

### Launching the example

To launch the monoDrive examples create 3 tabs and run each command in a separate terminal:

1. Launch rosbridge, you can leave this running: 

    ```bash
    roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True
    ```
2. Start the vehicle control node which will subscribe to the state sensor topic and publish vehicle controls (the simulator does not need to be running)

    ```bash
    rosrun vehicle_control node
    ```
    *Note: The vehicle_control example only requires the monodrive_msgs package and provides an example of how to connect your code to monoDrive through ROS messages.*

3. Make sure the monoDrive simulator is running since the next command will connect to and start the simulator scenario running.

    ```bash
    rosrun simulator_control node
    ```