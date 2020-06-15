# LabVIEW Client Examples

After installing the monoDrive Client from the VI package manager, several
pre-built examples are available through the "Help" -> "Find Examples" 
dialog in LabVIEW. The NI monoDrive Project file, *ni_monoDrive.lvproj*, contains references to all the VI examples described below.

For all of these examples, you will need to have either run the monoDrive Simulator
or monoDrive Scenario Editor.

## Pilot Example

The Pilot Example, *mono1_pilot_example.vi*, enables users to control the vehicle's steering, throttle, brakes, starting position, and weather simulation in real time. 

*This is a free driving scenario with one car only.*

<div class="img_container">
    <img class="wide_img" src="../imgs/mono1.png" />
</div>

Prior to running the example, the user will be able to:

* **Select and change the weather profile** by changing the Weather Profile Control. There are various permutations of weather available in the monoDrive Simulator. 

* **Select the map** by using the Map Selection Drop-Down. When running, the monoDrive simulator will switch to this map prior to beginning the simulation.

* **Select a different vehicle start position** using the Start Position Control. Each of monoDrive's
maps are built with starting position. The number in this box corresponds with the number on the map display in the bottom right corner of the VI window.

* **For modifications** to this example, use the pilot_example.json file which can be found in the installation directory under: 

    `C:\Program Files\National Instruments\LabVIEW 2019\vi.lib\monoDrive\monoDriveClient\labview\scenarios`


To run the example, **Click on the "Run" arrow on the top-left corner to start the client.** This will connect the client to the simulator. Errors during the connection will appear in the "Error" dialog on the right. Once connected, the following actions are available through the GUI:

* **Move the car** using the Throttle Slider. This control shows the percentage of 
the throttle the vehicle will be using.

* **Change the direction** using Direction Button. Switch 
between moving the vehicle forward or reverse.

* **Steer the vehicle** with the Steering Control. This controls the angle of 
turning.

* **Apply brakes** using the Brake Button. This will apply 100% of the brakes 
to the vehicle and will stop the vehicle.

**Configuration information:** [mono1_pilot_example.vi](../../examples/mono1_pilot_example)

<p>&nbsp;</p>

## Scenario Example

The Scenario Example, mono2_scenario_example.vi, enables users to control the vehicle's steering, throttle, brakes, starting position, and weather simulation in real time. 

*This is a free driving scenario with other vehicles spawning on specified positions in autopilot mode*

<div class="img_container">
    <img class="wide_img" src="../imgs/mono2.png" />
</div>

Prior to running the example, the user will be able to:

* **Select and change the weather profile** by changing the Weather Profile Control. There are various permutations of weather available in the monoDrive Simulator. 

* **Select the map** by using the Map Selection Drop-Down. When running, the monoDrive simulator will switch to this map prior to beginning the simulation.

* **For modifications** to this example, use the scenario_example.json file which can be found in the installation directory under: 

    `C:\Program Files\National Instruments\LabVIEW 2019\vi.lib\monoDrive\monoDriveClient\labview\scenarios`


To run the example, **Click on the "Run" arrow on the top-left corner to start the client.** This will connect the client to the simulator. Errors during the connection will appear in the "Error" dialog on the right. Once connected, the following actions are available through the GUI:

* **Move the car** using the Throttle Slider. This control shows the percentage of 
the throttle the vehicle will be using.

* **Change the direction** using Direction Button. Switch 
between moving the vehicle forward or reverse.

* **Steer the vehicle** with the Steering Control. This controls the angle of 
turning.

* **Apply brakes** using the Brake Button. This will apply 100% of the brakes 
to the vehicle and will stop the vehicle.

**Configuration information:** [mono2_scenario_example.vi](../../examples/mono2_scenario_example)

<p>&nbsp;</p>

## Replay Example

The Replay Example, *mono_replay_example.vi*, enables users to select and run a monoDrive Trajectory File from beginning to the end. This example does not have the ability to stop at certain point, but is available in the Radar Example. 

<div class="img_container">
    <img class="wide_img" src="../imgs/mono3.png" />
</div>

Prior to running the example, the user will be able to:

* **Select and change the weather profile** by changing the Weather Profile Control. There are various permutations of weather available in the monoDrive Simulator. 

* **Select the map** by using the Map Selection Drop-Down. When running, the monoDrive simulator will switch to this map prior to beginning the simulation.

* **Select the trajectory file** by using the Trajectory File Selection dialog. 
Successfully connecting the client, this trajectory will be sent to the simulator and immediately begin playing. To change the trajectory file, click the 
"Browse" button the right-hand side of the dialog. Pre-configured trajectories 
can be found in the installation directory under: 

`C:\Program Files\National Instruments\LabVIEW 2019\vi.lib\monoDrive\monoDriveClient\labview\trajectories`

To run the example, **Click on the "Run" arrow on the top-left corner to start the client.** The playback will begin and the current frame steps will be shown in the "Current Frame" indicator.

**Configuration information:** [mono3_replay_example.vi](../../examples/mono3_replay_example)

<p>&nbsp;</p>


## HIL Example

The HIL (Hardware-in-the-loop) Example, *mono4_hil_example.vi*, enables users to control the "ego" vehicle. All vehicles will be positioned in provided time steps in the simulation. To trigger the movement, users will need to provide specific direct hardware inputs or software in order to move the ego vehicle. The "EGO Pose Controls" are provided as an example and intended to be replaced by the user's own code.

<div class="img_container">
    <img class="wide_img" src="../imgs/mono4.png" />
</div>

Prior to running the example, the user will be able to:

* **Select and change the weather profile** by changing the Weather Profile Control. There are various permutations of weather available in the monoDrive Simulator. 

* **Select the map** by using the Map Selection Drop-Down. When running, the monoDrive simulator will switch to this map prior to beginning the simulation.

* **Select the trajectory file** by using the Trajectory File Selection dialog. 
Successfully connecting the client, this trajectory will be sent to the simulator and immediately begin playing. To change the trajectory file, click the 
"Browse" button the right-hand side of the dialog. Pre-configured trajectories 
can be found in the installation directory under: 

`C:\Program Files\National Instruments\LabVIEW 2019\vi.lib\monoDrive\monoDriveClient\labview\trajectories`

* **Control the EGO vehicle position** with the EGO Pose Controls. The changes 
will immediately take effect on the EGO position and kinematics.

**Configuration information:** [mono4_hil_example.vi](../../examples/mono4_hil_example)

<p>&nbsp;</p>

## Radar Example

The Radar Example, *mono5_radar_example.vi*, enables users to select and run a monoDrive Trajectory File from beginning to the end. The difference between Radar Example and Replay Example, is that this example has the ability to stop at any point during the simulation so that a user may observe or modify the simulation more easily. This is especially helpful when testing a radar sensor.

<div class="img_container">
    <img class="wide_img" src="../imgs/mono5.png" />
</div>

Prior to running the example, the user will be able to:

* **Select and change the weather profile** by changing the Weather Profile Control. There are various permutations of weather available in the monoDrive Simulator. 

* **Select the map** by using the Map Selection Drop-Down. When running, the monoDrive simulator will switch to this map prior to beginning the simulation.

* **Select the trajectory file** by using the Trajectory File Selection dialog. 
Successfully connecting the client, this trajectory will be sent to the simulator and immediately begin playing. To change the trajectory file, click the 
"Browse" button the right-hand side of the dialog. Pre-configured trajectories 
can be found in the installation directory under: 

`C:\Program Files\National Instruments\LabVIEW 2019\vi.lib\monoDrive\monoDriveClient\labview\trajectories`

To run the example, **Click on the "Run" arrow on the top-left corner to start the client.** The playback will begin and the current frame steps will be shown in the "Current Frame" indicator.

To stop the simulation at a specific point or to go through the scenario frame-by-frame, **use the Autoplay Toggle**. When this toggle is on, the trajectory will be advanced through the simulation without stopping like the Replay Example. When switched off the simulation will stop, use the "Next" button or the "Previous" button to control the frame.

* **"Next" button** will advance the simulation forward one step towards the end of the trajectory file.

* **"Previous" button** will return to the previous step towards the beginning of the trajectory file.

**Configuration information:** [mono5_radar_example.vi](../../examples/mono5_radar_example)

<p>&nbsp;</p>

## Pilot Example with Fixed Time

The Pilot Example with fixed time, *mono6_pilot_example_fixed_time.vi*, enables users to control the vehicle's steering, throttle, brakes, starting position, and weather simulation with fixed time step. 


<div class="img_container">
    <img class="wide_img" src="../imgs/mono6.png" />
</div>

Prior to running the example, the user will be able to:

* **Select and change the weather profile** by changing the Weather Profile Control. There are various permutations of weather available in the monoDrive Simulator. 

* **Select the map** by using the Map Selection Drop-Down. When running, the monoDrive simulator will switch to this map prior to beginning the simulation.

* **Select a different vehicle start position** using the Start Position Control. Each of monoDrive's
maps are built with starting position. The number in this box corresponds with the number on the map display in the bottom right corner of the VI window.

* **For modifications** to this example, use the pilot_example.json file which can be found in the installation directory under: 

    `C:\Program Files\National Instruments\LabVIEW 2019\vi.lib\monoDrive\monoDriveClient\labview\scenarios`

To run the example, **Click on the "Run" arrow on the top-left corner to start the client.** This will connect the client to the simulator. Errors during the connection will appear in the "Error" dialog on the right. Once connected, the following actions are available through the GUI:

* **Move the car** using the Throttle Slider. This control shows the percentage of 
the throttle the vehicle will be using.

* **Change the direction** using Direction Button. Switch 
between moving the vehicle forward or reverse.

* **Steer the vehicle** with the Steering Control. This controls the angle of 
turning.

* **Apply brakes** using the Brake Button. This will apply 100% of the brakes 
to the vehicle and will stop the vehicle.

**Configuration information:** [mono6_pilot_example_fixed_time.vi](../../examples/mono6_pilot_example_fixed)

<p>&nbsp;</p>


## Experimental Examples

Experimental examples show one of many ways to integrate custom code into the LabVIEW client. These examples are in the *experimental* stage and can be found at:

`C:\Program Files\National Instruments\LabVIEW 2019\vi.lib\monoDrive\monoDriveClient\labview\experimental\`

Follow the `README.md` located in the `experimental` folder prior to running these examples in order to compile and install the necessary DLLs. 


### Lane Follower Example 

The Lane Follower Example, *mono_lane_follower_cpp_dll.vi*, demonstrates a vehicle's ability to follow a lane using a compiled C++ DLL. This algorithm uses GeoJSON map and state sensor information to follow a second lane on the map.

<div class="img_container">
    <img class='wide_img' src="../imgs/lane_follower.png"/>
</div>

Ensure the map and trajectory file are properly selected prior to running the client. To run the example, click on the "Run" arrow on the top-left corner to start the client. The playback will begin by executing the first frame of the modified trajectory.

* **NOTE: The Steering Control is disabled in this example.** The steering value is being calculated by the C++ compiled DLL.

* **Move the car** using the Throttle Slider. This control shows the percentage of the throttle the vehicle will be using.

* **Change the direction** using Direction Button. Switch between moving the vehicle forward or reverse.

* **Apply brakes** using the Brake Button. This will apply 100% of the brakes to the vehicle and will stop the vehicle.

<p>&nbsp;</p>

### Radar with Kalman Filter Example 

The Radar with Kalman Filtering Example, *mono_radar_with_kalman_filter_cpp_dll.vi*, demonstrates a compiled C++ DLL with implementation of a Kalman Filter. The Kalman Filter applies to the Angle of Arrival from monoDrive's Radar sensor when the EGO vehicle brakes during an Automatic Emergency Braking (AEB) scenario. This example is intended to be used with the monoDrive Simulator's "Highway Track" map and the "Car-to-Car-Rear-Stationary.json" trajectory.

<div class="img_container">
    <img class='wide_img' src="../imgs/kalman.png"/>
</div>

Ensure the map and trajectory file are properly selected prior to running the client. To run the example, click on the "Run" arrow on the top-left corner to start the client. The playback will begin by
executing the first frame of the modified trajectory. As the vehicle approaches the stationary vehicle in front of it, the brakes will be dynamically applied by the Kalman filtering algorithm.

<p>&nbsp;</p>