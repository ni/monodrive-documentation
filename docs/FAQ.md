# F.A.Q.   

<h2> Logs </h2>

- [ If an error occurred while build or a crash ocurred, where can I find a log for these?](./#find-error-log)
- [When I tried to build the scenario Editor, I get the following error on the logs: ```libboost_date_time-vc141-mt-x64-1_74.lib```](./#error-log-messages)
- [When I tried to build the scenario Editor, I see the following error on the logs: ```'cuda_runtime.h' file not found```](./#error-log-messages)

<h2> Installation </h2>

- [After some time building, I see the VehicleAI window stuck at 95%](./#vehicleai-window-stuck-at-95)

- [When building the C++ Client, I get the error: "Unable to determine what CMake generator to use. Please install or configure a preferred generator, or update settings.json](./#cmake-error)

- [When I try building the Linux Scenario Editor, I receive this error message even though the plugins were extracted in the correct location: "The following modules are missing...."](./#modules-missing-error-linux)

<h2> Licensing </h2>

- [I moved my license to a new machine, the simulator shows an ```UNLICENSED. License in use``` message, even though my license is still valid.](./#unlicensed-license-in-use-message)
- [The simulator shows an ```License is not found``` message.](./#the-simulator-shows-an-license-is-not-found-message)

- [I received this error message, "'Scenario Editor Licence Error' - the feature 'monodrive_scenario_editor' is not available", and I have a valid license in the correct location.](./#scenario-editor-licence-error-simulator-license-error-windows)

<p>&nbsp;</p>

_ _ _ 

## Logs

##### Find Error Log

If an error occurred while build or a crash ocurred, where can I find a log for these?

&emsp; monoDrive saves a log for Scenario Editor and the simulator on the following locations:

 - Scenario Editor   
    - `Installation Folder\VehicleAI_Editor\Saved\Logs\VehicleAI.log`

 - Simulator   
    - Crash logs: `C:\Users\{user}\AppData\Local\VehicleAI\Saved\Crashes`

##### Error Log Messages

When I tried to build the scenario Editor, I get the following error on the logs: ```libboost_date_time-vc141-mt-x64-1_74.lib```

- This error is due to having an older version of Visual Studio. Ensure you have Visual Studio 2019, see [prerequisites](Getting_Started.md). 

When I tried to build the scenario Editor, I see the following error on the logs: ```'cuda_runtime.h' file not found```

-  This error is due to missing the Cuda SDK missing. Ensure you install from [here](https://developer.nvidia.com/cuda-10.2-download-archive). More information see [prerequisites](Getting_Started.md).   
<p>&nbsp;</p>
_ _ _ 

## Installation

##### VehicleAI window stuck at 95%

After some time building, I see the VehicleAI window stuck at 95%.

monoDrive's Scenario Editor has high quality graphics that will take some time to compile when opening for the first time. Verify the process is still running by opening the Task Manager and see shaders are still compiling.

##### CMake Error

When building the C++ Client, I get the error: "Unable to determine what CMake generator to use. Please install or configure a preferred generator, or update settings.json

To use CMake, you will need to install [Visual Studio 2019 Community Edition](https://visualstudio.microsoft.com/vs/community/). See [C++ Client Prerequisites](./cpp_client/cpp_quick_start.md)

##### Modules Missing Error: Linux

<div class="img_container">
    <img class='lg_img' src="../imgs/monodrive_error.png"/>
</div>

The VehicleAI project needs to be rebuilt for your build of the Unreal Engine. To do this you will need to run a few more commands:

1. Generate updated project files for the editor:

```
${UE4_ROOT}/GenerateProjectFiles.sh -project="${SIMULATOR_ROOT}/VehicleAI.uproject" -game -engine -makefiles
```

2. Build the editor:
```
cd ${SIMULATOR_ROOT}
make VehicleAIEditor
```

3. Run the editor like normal:
```
${UE4_ROOT}/Engine/Binaries/Linux/UE4Editor ${SIMULATOR_ROOT}/VehicleAI.uproject
```
<p>&nbsp;</p>
_ _ _ 

## Licensing 

##### "UNLICENSED. License in use" message

I moved my license to a new machine, the simulator shows an ```UNLICENSED. License in use``` message, even though my license is still valid.

- Each license is for one machine's use only. 
- If you did any change to the hardware,i.e. connect to a docking station, change ethernet card, etc. Your license can be blocked as well. 
- Please contact us through email at **support@monodrive.io** or through a communication channel already established, so we can help you resolve this issue. 

##### The simulator shows an "License is not found" message.

- Ensure that you have copied a valid `license.txt` file into the correct folder, and that the name of the file is `license.txt`. It should be next to the VehicleAI.exe or the VehicleAI.uproject.   

##### Scenario Editor Licence Error/ Simulator License Error (Windows)

I received this error message, "'Scenario Editor Licence Error' - the feature 'monodrive_scenario_editor' is not available", and I have a valid license in the correct location.

 - Check to see if the simulator is able to connect to the server for license checks. [Click here to check connectivity](https://api.monodrive.io/api/v1/status), if there is no connectivity
    - Ensure that you are online for access
    - Check the firewall settings on your Windows machine is not blocking access to the network for vehicleAI / UE4Editor.exe. 
- Check the Logs to see if there is another issue, you can find the logs [here]((./#find-error-log)).

<p>&nbsp;</p>
