# Scenario Editor

## Prerequisites 

1. Ensure that [Visual Studio 2019 or 2022](https://visualstudio.microsoft.com/downloads/) is installed to generate the project files. 

1. Install [Cuda Toolkit 11.6](https://developer.nvidia.com/cuda-11-6-2-download-archive)
    - During installation under "Options", "Express" will update or download the NVIDIA driver for this version of Cuda Toolkit. 
    - (Optional) If you have already updated to the most recent NVIDIA display drivers, you may want to customize this install under "Options", then "Custom". The next window will display installation options including "Driver Components", where you may remove checks to keep the driver version you currently have.

<div class ='img_container'>

<div class="img_container">
    <img class='md_img' src="../../../imgs/cuda_options.png"/>
</div>

<div class="img_container">
    <img class='md_img' src="../../../imgs/cuda_options2.png"/>
</div>
  
</div>

## Windows Scenario Editor

1. Extract the archive to a common location like `C:/monodrive`.
1. If downloading for the first time, you will need to download the license.txt file through your account on [monoDrive.io](https://www.monodrive.io/register), unless one has been provided directly from monoDrive. Copy the attached license.txt file to the extracted location `C:/monodrive/VehicleAI_Editor/license.txt`.
1. Install [Unreal Engine 4.25.4](https://www.unrealengine.com/en-US/).
1. Extract the Plugins.zip archive into the 4.25.4 Engine's Plugins directory. e.g. if UE4.25.4 is installed at `C:\Program Files\Epic Games\UE_4.25`, then extract the archive into `C:\Program Files\Epic Games\UE_4.25\Engine\Plugins`. The resulting directory structure should look as follows:
    <pre>
        C:\Program Files\Epic Games\UE_4.25\Engine\Plugins\monoDrive
            +-- monoDriveSensors
            +-- ... (other monoDrive plugins)
    </pre>         
5. Open VehicleAI.uproject. Note that the first time you open the project, UE4 will prompt you about missing modules that need to be rebuilt (VehicleAI and VehicleAIEditor). Select "Yes" to build the modules, then the project will begin compiling shaders. 

    Note that the editor may take a long time to compiling shaders, and may appear to get stuck at 95% or 99%. This is expected and you will need to wait for the compiling to finish. If no error is reported then the system is still compiling shaders, this only happens the first time opening. 

    Note that the Simulator needs access to communicate to Public networks, select "Allow Access". If this window does not pop-up and there is a license error, see the [solution here](../../../FAQ/#scenario-editor-licence-error-simulator-license-error-windows).
    
    <div class="img_container">
    <img class='lg_img' src="../../img/allow_access.png"/>
    </div>

## Windows Generating Project Files (Optional)

Generation of project files is not a requirement to run the Simulator, but it enables users to make changes to the open source from monoDrive as well as run the debugger. To run the Simulator, go to your VehicleAI_Editor directory and double-click on VehicleAI.uproject.

1. Download Visual Studio from [here](https://visualstudio.microsoft.com/).

    - Make sure the following options are enabled:

         - Workloads: Game development with C++

            - Options: C++ Profiling Tools

    *For more information, check [here](https://docs.unrealengine.com/en-US/Programming/Development/VisualStudioSetup/index.html)*

1. (Optional) For CarSim or VehicleSim Dynamics support, download CarSim from the Epic Games Launcher, select Marketplace >> Browse; search for "VehicleSim Dynamics." This can also be found from Epic Games website, sign-in and download the plugin [here](https://www.unrealengine.com/marketplace/en-US/product/carsim-vehicle-dynamics).

1. Generate Visual Studio project files by right clicking on VehicleAI.uproject in the VehicleAI directory. 

    <div class="img_container">
    <img class='lg_img' src="../../../LV_client/quick_start/imgs/generate_project_files.png"/>
    </div>

2. Double-click on VehicleAI.sIn to open the Simulator from Visual Studio.

    <div class="img_container">
    <img class='lg_img' src="../../../LV_client/quick_start/imgs/vehicle-sIn.png"/>
    </div>

3. Run the Scenario Editor.

    <div class="img_container">
    <img class='wide_img' src="../../../LV_client/quick_start/imgs/play.png"/>
    </div>

## Update Versions

1. Extract the new archive to a common location like `C:/monodrive`.
1. Copy your license.txt file to the new extracted location `C:/monodrive/VehicleAI_Editor/license.txt`.
1. Delete previous monoDrive Plugins and extract the new verion of Plugins.zip archive into the 4.25.4 Engine's Plugins directory. e.g. if UE4.25.4 is installed at `C:\Program Files\Epic Games\UE_4.25`, then extract the archive into `C:\Program Files\Epic Games\UE_4.25\Engine\Plugins`. The resulting directory structure should look as follows:
    <pre>
        C:\Program Files\Epic Games\UE_4.25\Engine\Plugins\monoDrive
            +-- monoDriveSensors
            +-- ... (other monoDrive plugins)
    </pre>         
5. Open VehicleAI.uproject. Note that the first time you open the project, UE4 will prompt you about missing modules that need to be rebuilt (VehicleAI and VehicleAIEditor). Select "Yes" to build the modules, then the project will begin compiling shaders. 

## Open Source Clients

Working with the monoDrive Simulator can be done directly through TCP/IP communications with any application that conforms to the monoDrive API. However, monoDrive provides several reference implementations of clients in C++, LabVIEW, and Python. The monoDrive Clients are open source and available to all users of the monoDrive Simulator and Scenario Editor.

 - Go to [C++ Client Setup](../../cpp_client/cpp_quick_start.md)

 - Go to [LabVIEW Client Setup](../../LV_client/quick_start/LabVIEW_client_quick_start.md)

 - Go to [Python Client Setup](../../python_client/quick_start.md)

## Installation & Licensing Errors

For frequently asked questions, see our FAQ Page. If this  does not resolve the seen issue, please get in contact with [ NI monoDrive support team](https://www.monodrive.io/contact).

 - [FAQ Installation](../../../FAQ/#installation)
 - [FAQ Licensing](../../../FAQ/#licensing)

<p>&nbsp;</p>