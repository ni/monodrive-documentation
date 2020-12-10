# Windows

[Download monoDrive](https://www.monodrive.io/register)

## Simulator

1. Extract the archive to a common location like `C:/monodrive`.
1. If downloading for the first time, you will need to download the license.txt file through your account on [monoDrive.io](https://www.monodrive.io/register), unless one has been provided directly from monoDrive. Copy the attached license.txt file to the extracted location `C:/monodrive/VehicleAI/license.txt`.
1. Run the simulator by launching `C:/monodrive/VehicleAI/VehicleAI.exe`.

## Scenario Editor

1. Extract the archive to a common location like `C:/monodrive`.
1. If downloading for the first time, you will need to download the license.txt file through your account on [monoDrive.io](https://www.monodrive.io/register), unless one has been provided directly from monoDrive. Copy the attached license.txt file to the extracted location `C:/monodrive/VehicleAI_Editor/license.txt`.
1. Install [Unreal Engine 4.24.3](https://www.unrealengine.com/en-US/).
1. Extract the Plugins.zip archive into the 4.24.3 Engine's Plugins directory. e.g. if UE4.24.3 is installed at `c:\Program Files\Unreal\UE_4.24.3`, then extract the archive into `c:\Program Files\Unreal\UE_4.24.3\Engine\Plugins`. The resulting directory structure should look as follows:
<pre>
    c:\Program Files\Unreal\UE_4.24.3\Engine\Plugins\monoDrive
        +-- monoDriveRadarSensor
        +-- monoDriveLidarSensor
        +-- ... (other monoDrive plugins)
    </pre>         
5. Open VehicleAI.uproject. Note that the first time you open the project, UE4 will prompt you about missing modules that need to be rebuilt (VehicleAI and VehicleAIEditor). Select "Yes" to build the modules, then the project will open.


## Scenario Editor: Generating Project Files (Optional)

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

    <p>&nbsp;</p>

    Go to [LabVIEW Client Setup](../../LV_client/quick_start/LabVIEW_client_quick_start.md)
    
    Go to [C++ Client Setup](../../cpp_client/cpp_quick_start.md)

    Go to [Python Client Setup](../../python_client/quick_start.md)
    
    <p>&nbsp;</p>