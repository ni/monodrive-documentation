# F.A.Q.   

<h2> Logs </h2>

- [ If an error occurred while build or a crash ocurred, where can I find a log for these?](./#find-error-log)
- [When I tried to build the scenario Editor, I get the following error on the logs: ```libboost_date_time-vc141-mt-x64-1_74.lib```](./#error-log-messages)
- [When I tried to build the scenario Editor, I see the following error on the logs: ```'cuda_runtime.h' file not found```](./#error-log-messages)

<h2> Installation </h2>

- [After some time building, I see the VehicleAI window stuck at 95%](./#vehicleai-window-stuck-at-95)

<h2> Licensing </h2>

- [I moved my license to a new machine, the simulator shows an ```UNLICENSED. License in use``` message, even though my license is still valid.](./#unlicensed-license-in-use-message-even-though-license-is-still-valid)
- [The simulator shows an ```License is not found``` message.](./#the-simulator-shows-an-license-is-not-found-message)
<p>&nbsp;</p>

_ _ _ 

## Logs

##### Find Error Log

&emsp; monoDrive saves a log for Scenario Editor and the simulator on the following locations:

 - Scenario Editor   
    - `Installation Folder\VehicleAI_Editor\Saved\Logs\VehicleAI.log`

 - Simulator   
    - Crash logs: `C:\Users\{user}\AppData\Local\VehicleAI\Saved\Crashes`

##### Error Log Messages
- ```libboost_date_time-vc141-mt-x64-1_74.lib```
    - This error is due to having an older version of Visual Studio. Ensure you have Visual Studio 2019, see [prerequisites](Getting_Started.md). 

- ```'cuda_runtime.h' file not found```
    -  This error is due to missing the Cuda SDK missing. Ensure you install from [here](https://developer.nvidia.com/cuda-10.2-download-archive). More information see [prerequisites](Getting_Started.md).   
<p>&nbsp;</p>
_ _ _ 

## Installation

##### VehicleAI window stuck at 95%

monoDrive's Scenario Editor has high quality graphics that will take some time to compile when opening for the first time. Verify the process is still running by opening the Task Manager and see shaders are still compiling.
<p>&nbsp;</p>
_ _ _ 

## Licensing 

##### "UNLICENSED. License in use" message, even though license is still valid.

- Each license is for one machine's use only. 
- If you did any change to the hardware,i.e. connect to a docking station, change ethernet card, etc. Your license can be blocked as well. 
- Please contact us through email at **support@monodrive.io** or through a communication channel already established, so we can help you resolve this issue. 

##### The simulator shows an "License is not found" message.

- Ensure that you have copied a valid `license.txt` file into the correct folder, and that the name of the file is `license.txt`. It should be next to the VehicleAI.exe or the VehicleAI.uproject.   
<p>&nbsp;</p>


