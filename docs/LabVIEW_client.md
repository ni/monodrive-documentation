# LabVIEW Client
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/docs/LV_client/closed_loop_FP.jpg" width="400"  />
</p>


### Setup
1. Download LabVIEW 2018 (32 bit) from this [website](http://www.ni.com/download/labview-development-system-2018/7406/en/).
2. Download VeloView from this [website](https://www.paraview.org/download/).
3. Download the monoDrive Simulator from [here](https://www.monodrive.io/download).
4. Download the monoDrive Client from the VIPM (installed with LabVIEW). Look for *monoDrive* on the VIPM browser. Install a compatible version for the simulator you have, i.e. Simulator 1.4.x.x with client 1.4.x.x

**NOTE:**
We recommend this hardware setup to run the simulator and the client with the best performance.
Windows 10 running on Intel Core i9-9900 or better with NVIDIA RTX GPU and minimum 16GB RAM

### Description 

This example demonstrates how to use LabVIEW as a client for the monoDrive Simulator.

#### Instructions:

1. Make sure you download the monoDrive Simulator from https://www.monodrive.io/download
2. Run the **VehicleAI.exe**


#### To Run in Closed loop mode:
1. Make sure you the simulator is running before you run the *monoDrive_closed_loop_example.vi*.
2. Click on the top left arrow to run.
3. Use the steering control and the forward control to move the car.
4. Open the block diagram and click on the sensors VI, to look at the output.


#### To Run in Replay mode:

1. Change the Trajectory Config path with a replay configuration. i.e. LeftTurnCrossWalk.json 
2. Click on the top left arrow to run.
3. Open the block diagram and click on the sensors VI, to look at the output.


#### To Run in Replay_step mode:
1. Change the Trajectory Config path with a replay configuration. i.e. LeftTurnCrossWalk.json
2. Click on the top left arrow to run.
3. Open the block diagram and click on the sensors VI, to look at the output.

For technical support contact us at <b>support@monodrive.io</b>
