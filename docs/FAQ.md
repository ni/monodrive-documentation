# F.A.Q.   

<h2> Logs </h2>

- [ If an error occurred while build or a crash ocurred, where can I find a log for these?](./#find-error-log)
- [When I tried to build the scenario Editor, I get the following error on the logs: ```libboost_date_time-vc141-mt-x64-1_74.lib```](./#error-log-messages)
- [When I tried to build the scenario Editor, I see the following error on the logs: ```'cuda_runtime.h' file not found```](./#error-log-messages)

<h2> Installation </h2>

- [After some time building, I see the VehicleAI window stuck at 95%](./#vehicleai-window-stuck-at-95)
- [Generate Project Files option is missing](./#generate-project-files-option-is-missing)
- [Error Message: VehicleAI could not be compiled. Try rebuilding from source manually.](./#error-vehicleai-could-not-be-compiled-try-rebuilding-from-source-manually)
- [Error Message: DirectX Runtime Error / Scenario Editor Crash with D12RHI References](./#error-directx-runtime-error-scenario-editor-crash-with-d12rhi-references)

- [When building the C++ Client, I get the error: "Unable to determine what CMake generator to use. Please install or configure a preferred generator, or update settings.json](./#cmake-error)

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

-  This error is due to missing the Cuda SDK missing. Ensure you install [Cuda Toolkit 11.6](https://developer.nvidia.com/cuda-11-6-2-download-archive). More information see [prerequisites](Getting_Started.md).   
<p>&nbsp;</p>
_ _ _ 

## Installation

##### VehicleAI window stuck at 95%

After some time building, I see the VehicleAI window stuck at 95%.

monoDrive's Scenario Editor has high quality graphics that will take some time to compile when opening for the first time. Verify the process is still running by opening the Task Manager and see shaders are still compiling.

##### CMake Error

When building the C++ Client, I get the error: "Unable to determine what CMake generator to use. Please install or configure a preferred generator, or update settings.json

To use CMake, you will need to install [Visual Studio 2019 Community Edition](https://visualstudio.microsoft.com/vs/community/). See [C++ Client Prerequisites](./cpp_client/cpp_quick_start.md)

<p>&nbsp;</p>

##### Generate Project Files option is missing 

<div class="img_container">
   <img class='lg_img' src="../imgs/faq_generate.png"/>
</div>

You may need to permanently associate the project with Unreal Engine by opening the .uproject first before generating the project files. Then after rebooting, the "generate project files" may show as an option then. If this is not the case, try this solution offered from Unreal Engine Support:

Set the file association of the “.uproject” file to “C:\Program Files (x86)\Epic Games\Launcher\Engine\Binaries\Win64\UnrealVersionSelector.exe” by following these steps:

   1. right-click the .uproject file
   1. click “open with…”
   1. click “choose another app”
   1. click “more apps”
   1. click “look for another app”
   1. browse to “C:\Program Files (x86)\Epic Games\Launcher\Engine\Binaries\Win64”
   1. select “UnrealVersionSelector.exe”
   1. click “open”

<p>&nbsp;</p>

##### Error: VehicleAI could not be compiled. Try rebuilding from source manually.

<div class="img_container">
   <img class='lg_img' src="../imgs/faq_rebuild.png"/>
</div>

Usually this popup error indicates that you may need to re-extract or extract the monoDrive Plugins, below are instructions on how to do this. 

Delete previous monoDrive Plugins (if there are any), and extract the Plugins.zip archive from the root of the Scenario Editor into the 4.27.2 Engine's Plugins directory. e.g. if UE4.27.2 is installed at `C:\Program Files\Epic Games\UE_4.27`, then extract the archive into `C:\Program Files\Epic Games\UE_4.27\Engine\Plugins`. The resulting directory structure should look as follows:
    <pre>
        C:\Program Files\Epic Games\UE_4.27\Engine\Plugins\monoDrive
            +-- monoDriveSensors
            +-- ... (other monoDrive plugins)
    </pre>         
Rerunning Visual Studio is required to see changes. 

##### Error: DirectX Runtime Error / Scenario Editor Crash with D12RHI References

<div class="img_container">
   <img class='lg_img' src="../imgs/faq_dx11_2.png"/>
</div>

DirectX11 is required to run monoDrive Simulator or Scenario Editor on Windows. You can [download DirectX11](https://www.microsoft.com/en-us/Download/confirmation.aspx?id=35) if working with the Simulator.

If working with the Editor, you may resolve the issue by setting DirectX11 in Unreal Engine through Edit --> Project Settings. In the Details search bar, search "directx", there you should see a setting to change to DirectX11. 

<div class="img_container">
   <img class='lg_img' src="../imgs/faq_dx11.png"/>
</div>

_ _ _ 

## Licensing 

##### "UNLICENSED. License in use" message

I moved my license to a new machine, the simulator shows an ```UNLICENSED. License in use``` message, even though my license is still valid.

- Each license is for one machine's use only. 
- If you did any change to the hardware,i.e. connect to a docking station, change ethernet card, etc. Your license can be blocked as well. 
- Please contact us through email at **monodrive.support@ni.com** or through a communication channel already established, so we can help you resolve this issue. 

##### The simulator shows an "License is not found" message.

- Ensure that you have copied a valid `license.txt` file into the correct folder, and that the name of the file is `license.txt`. It should be next to the VehicleAI.exe or the VehicleAI.uproject.   

##### Scenario Editor Licence Error/ Simulator License Error (Windows)

I received this error message, "'Scenario Editor Licence Error' - the feature 'monodrive_scenario_editor' is not available", and I have a valid license in the correct location.

 - Check to see if the simulator is able to connect to the server for license checks. [Click here to check connectivity](https://api.monodrive.io/api/v1/status), if there is no connectivity
    - Ensure that you are online for access
    - Check the firewall settings on your Windows machine is not blocking access to the network for vehicleAI / UE4Editor.exe. 
- Check the Logs to see if there is another issue, you can find the logs [here]((./#find-error-log)).

<p>&nbsp;</p>
