# LabVIEW Client Setup

<p>&nbsp;</p>
<p class="img_container">
<img class="wide_img" src="https://github.com/monoDriveIO/client/raw/master/docs/LV_client/closed_loop_FP.jpg" />
</p>
<p>&nbsp;</p>


## Setup

1. Download LabVIEW 2018 (32 bit) from this [website](http://www.ni.com/download/labview-development-system-2018/7406/en/).

2. Download VeloView from this [website](https://www.paraview.org/download/).

3. Download the monoDrive Simulator from [here](https://www.monodrive.io/register).

4. Download the monoDrive Client from the VIPM (installed with LabVIEW). Look for *monoDrive* on the VIPM browser.


    **NOTE:**
    We recommend this hardware setup to run the simulator and the client with the best performance.  

    **Windows 10 running on Intel Core i9-9900 or better with NVIDIA RTX GPU and minimum 16GB RAM**

<p>&nbsp;</p>


## Description 

This example demonstrates how to use LabVIEW as a client for the monoDrive Simulator.

### Instructions:

1. Make sure you download the monoDrive Simulator from [monoDrive](https://www.monodrive.io/register)

2. Run the **VehicleAI.exe**
<p>&nbsp;</p>


### To Run in Closed Loop Mode:

1. Make sure you the simulator is running before you run the *monoDrive_closed_loop_example.vi* ( [see interface example](LV_client/monoDrive_Simulator_Interface_Example.md) ). 

2. Click on the top left arrow to run.

3. Use the steering control and the forward control to move the car.

4. Open the block diagram and click on the sensors VI, to look at the output.
<p>&nbsp;</p>



### To Run in Replay Mode:

1. Make sure you the simulator is running before you run the *monoDrive_replay_example.vi* ( [see interface example](LV_client/monoDrive_Simulator_Interface_Example.md) ). 

2. Change the Trajectory Config path with a replay configuration. i.e. LeftTurnCrossWalk.json 

3. Click on the top left arrow to run.

4. Open the block diagram and click on the sensors VI, to look at the output.
<p>&nbsp;</p>



### To Run in HIL Mode:

1. Make sure you the simulator is running before you run the *monoDrive_hil_example.vi* ( [see interface example](LV_client/monoDrive_Simulator_Interface_Example.md) ). 

2. Change the Trajectory Config path with a replay configuration. i.e. LeftTurnCrossWalk.json

3. Click on the top left arrow to run.

4. Open the block diagram and click on the sensors VI, to look at the output.

<p>&nbsp;</p>


For technical support contact us at <b>support@monodrive.io</b>
<p>&nbsp;</p>
