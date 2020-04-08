# monoDrive Client Examples

After installing the monoDrive Client from the VI package manager, several
pre-built examples will be available through the "Help" -> "Find Examples" 
dialog in LabView. The `ni_monoDrive.lvproj` contains references to all the VIs 
described here.

For all of these examples, you will need to have either the monoDrive Simulator
running or the monoDrive Scene Editor open and in "Play" mode.

## Closed Loop Example

The Closed Loop example (`mono_closed_loop_example.vi`) allows the user to 
control the vehicle's steering, throttle, brakes, and simulation weather in 
real time. 

<div class="img_container">
    <img class='wide_img' src="../imgs/CL_example.png" />
</div>

Prior to running the example, the user can:

* **Select the weather profile by changing selected weather profile using th 
Weather Profile Control.** There are various permutations of weather available 
in the monoDrive Simulator. 

* **Select the map by using the Map Selection Drop-down.** When ran, the 
monoDrive simulator will switch to this map prior to beginning the simulation.

* **Select a different position using the Start Position Control.** monoDrive 
maps come with various different start positions around the map. The number in 
this box corresponds with the number on the map displayed on the bottom right.

To run the example, **Click on the "Run" arrow on the top-left corner 
to start the client.** This will connect the client to the simulator. If there 
is an error during the connection it will appear in the "Error" dialog on the 
right. Once connected, the following actions are available through the GUI:

* **Move the car using the Throttle Slider.** This controls the percentage of 
the throttle the car will use.

* **Change the direction of travel using Direction control.** This will switch 
the vehicle between forward and reverse.

* **Steer the vehicle with the Steering Control** This controls the angle of 
steering.

* **Apply brakes using the Brake control.** This will apply 100% of the brakes 
to the vehicle and stop it.

## Replay Example

The Replay Example (`mono_replay_example.vi`) allows a user to select a 
monoDrive Trajectory File and play it back in simulation. Many of the controls 
and dialogs are similar to the the Closed Loop Example. 

<div class="img_container">
    <img class='wide_img' src="../imgs/replay_example.png"/>
</div>

Prior to running this example, the user can: 

* **Select the trajectory file by using the Trajectory File Selection dialog.** 
This trajectory will be sent to the simulator and immediately begin playing on 
successfully connecting the client. To change the trajectory file, click the 
"Browse" button the right-hand side of the dialog. Pre-configured trajectories 
can be found in the installation directory under: 
`C:\Program Files\National Instruments\LabVIEW 2019\vi.lib\monoDrive\monoDriveClient\labview\trajectories`

To run the example, **Click on the "Run" arrow on the top-left corner 
to start the client.** The playback will begin and the current trajectory steps
will be shown in the "Current trajectory" indicator.

## Replay Controls

The Replay Controls Example (`mono_replay_controls_example.vi`) adds extra
controls to the the Replay Example. This example demonstrates how the replay
of a monoDrive Simulator trajectory file can be stepped forward and backward
so the user may observe or modify the simulation during replay more easily.

<div class="img_container">
    <img class='wide_img' src="../imgs/replay_controls.png"/>
</div>

Similar to the Replay Example, ensure the map and trajectory file are properly 
selected prior to running the client. To run the example, **Click on the "Run" 
arrow on the top-left corner to start the client.** The playback will begin by
executing the first frame of the trajectory. The user can now:

* **Click the "Next" button in the Replay Step Controls.** This will advance the 
simulation by one step forward until the end of the trajectory file.

* **Click the "Previous" button in the Replay Step Controls.** This will go to
the previous step in the simulation until the beginning of trajectory file.

* **Click the Autoplay Toggle on (up) or off (down).** When this toggle is on, 
the trajectory will be advanced forward in time until the end of the trajectory 
file or until the toggle is turned off.

### HIL Example

The HIL (Hardware-in-the-loop) Example (`mono_hil_example.vi`) is very similar
to the Replay Example, but it allows the user to control the "ego" vehicle
during the replay. Like the replay example, all vehicles in the simulation
are positioned as expected at each timestep, but the user is expected to provide
inputs in order to move the ego vehicle. This can be done through direct 
hardware inputs or software to trigger the movement. The "EGO Pose Controls" are
provided as an example and intended to be replaced by the user's own code.

<div class="img_container">
    <img class='wide_img' src="../imgs/hil_example.png"/>
</div>

Similar to the Replay Example, ensure the map and trajectory file are properly 
selected prior to running the client. To run the example, **Click on the "Run" 
arrow on the top-left corner to start the client.** The playback will begin by
executing the first frame of the trajectory. The user can now:

* **Control the EGO vehicle position with the EGO Pose Controls.** The changes 
made to this control will immediately take effect on the EGO position and 
kinematics.

### Multi Vehicle Pose Update Example

Similar to the Replay Example, the Mutli-Vehicle Pose Update Example 
(`mono_multi_vehilce_pose_update_example.vi`) plays back a replay when connected 
to the simulator. Additionally, it allows the user to change the values in
the trajectory file for every vehicle in scene.

<div class="img_container">
    <img class='wide_img' src="../imgs/multi_vehicle.png"/>
</div>

Prior to running this example, the user can: 

* **Select the frame to change values for using the Frame Selection.** This 
toggles the frame step in the simulation to change values for.

* **Change the vehicle to change values for using the Vehicle Selection.** 
This requires the user to type in the vehicle name of interest into the dialog 
to change the values for.

* **Change the vehicle kinematics using the Vehicle Controls.** Similar to the 
HIL example, this will change the kinematics of the vehicle currently selected 
in the Vehicle Selection.

Similar to the Replay Example, ensure the map and trajectory file are properly 
selected prior to running the client. To run the example, **Click on the "Run" 
arrow on the top-left corner to start the client.** The playback will begin by
executing the first frame of the modified trajectory.

## Experimental Examples

These example is currently in the *experimental* stage and can be found here:
`C:\Program Files\National Instruments\LabVIEW 2019\vi.lib\monoDrive\monoDriveClient\labview\experimental\`

You will need to follow the `README.md` located in the `expermintal` folder 
prior to running these examples in order to compile and install the necessary 
DLLs.

### Lane Follower Example 

The Lane Follower Example (`mono_lane_follower_cpp_dll.vi`) demonstrates the
use of a Kalman filter on a vehicle's ability to stay in and follow a lane. This
example is intended to be used with the monoDrive Simulator's "Highway Track" 
map, but can be used in other maps.

<div class="img_container">
    <img class='wide_img' src="../imgs/lane_follow.png"/>
</div>

Similar to the Closed Loop Example, ensure the map and trajectory file are 
properly selected prior to running the client. To run the example, **Click on 
the "Run" arrow on the top-left corner to start the client.** The playback will 
begin by executing the first frame of the trajectory. The user can now:

* **NOTE: The Steering Control is disabled in this example.** This is because it 
is being controlled by the Kalman filtering algorithm.

* **Move the car using the Throttle Slider.** This controls the percentage of 
the throttle the car will use.

* **Change the direction of travel using Direction control.** This will switch 
the vehicle between forward and reverse.

* **Steer the vehicle with the Steering Control** This controls the angle of 
steering.

* **Apply brakes using the Brake control.** This will apply 100% of the brakes 
to the vehicle and stop it.

## Radar with Kalman Filter Example 

The Radar with Kalman Filtering Example 
(`mono_radar_with_kalman_filter_cpp_dll.vi`) demonstrates the use of a Kalman 
Filter to apply the EGO vehicle's brakes during an Automatic Emergency Braking 
(AEB) scenario. This example is intended to be used with the monoDrive 
Simulator's "Highway Track" map and the "Car-to-Car-Rear-Stationary.json" 
trajectory.

<div class="img_container">
    <img class='wide_img' src="../imgs/radar.png"/>
</div>

Similar to the Replay Example, ensure the map and trajectory file are properly 
selected prior to running the client. To run the example, **Click on the "Run" 
arrow on the top-left corner to start the client.** The playback will begin by
executing the first frame of the trajectory. As the vehicle approaches the
stationary vehicle in front of it, the brakes will be dynamically applied by
the Kalman filtering algorithm.
