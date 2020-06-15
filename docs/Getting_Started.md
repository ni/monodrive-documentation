# Getting Started

## Prerequisites 

1. Install Unreal Engine **4.24** from [here](https://www.unrealengine.com/en-US/).

1. Ensure you have the latest NVIDIA drivers for your [CUDA-enabled](https://developer.nvidia.com/cuda-gpus) graphics card.

	<div class="img_container">
    <img class='sm_img' src="../imgs/nvidia_driver2.png"/>
	<img class='semiwide_img' src="../imgs/nvidia_driver1.png"/>
    </div>

    **NOTE**
    Even if not prompted by the system, a restart is recommended to ensure the new drivers are correctly detected prior to running any simulations.

    In case the driver cannot be updated using the above method, please visit http://www.nvidia.com/Download/index.aspx to download the [latest driver](http://www.nvidia.com/Download/index.aspx).

<p>&nbsp;</p>

<hr width="70%"/>

<p>&nbsp;</p>

## Simulator & Scenario Editor

1. Download the monoDrive Simulator from [here](https://www.monodrive.io/register) or the monoDrive Scenario Editor from [here](https://www.monodrive.io/register). 

1. After downloading, extract all files to a common folder such as ~/monodrive/. In an email after downloading, copy the attached license.txt file inside the common folder the simulator version was downloaded (~/monodrive).

1. To run the Simulator, go to your VehicleAI_Editor directory and double-click on VehicleAI.uproject.

<p>&nbsp;</p>

## Scenario Editor: Generating Project Files

Generation of project files is not a requirement to run the Simulator, but it enables users to make changes to the open source from monoDrive as well as run the debugger. To run the Simulator, go to your VehicleAI_Editor directory and double-click on VehicleAI.uproject.

1. Download Visual Studio from [here](https://visualstudio.microsoft.com/).

    - Make sure the following options are enabled:

         - Workloads: Game development with C++

            - Options: C++ Profiling Tools

    *For more information, check [here](https://docs.unrealengine.com/en-US/Programming/Development/VisualStudioSetup/index.html)*

1. (Optional) For CarSim or VehicleSim Dynamics support, download CarSim from the Epic Games Launcher, select Marketplace >> Browse; search for "VehicleSim Dynamics." This can also be found from Epic Games website, sign-in and download the plugin [here](https://www.unrealengine.com/marketplace/en-US/product/carsim-vehicle-dynamics).

1. Generate Visual Studio project files by right clicking on VehicleAI.uproject in the VehicleAI directory. 

    <div class="img_container">
    <img class='lg_img' src="../LV_client/quick_start/imgs/generate_project_files.png"/>
    </div>

2. Double-click on VehicleAI.sIn to open the Simulator from Visual Studio.

    <div class="img_container">
    <img class='lg_img' src="../LV_client/quick_start/imgs/vehicle-sIn.png"/>
    </div>

3. Run the Scenario Editor.

    <div class="img_container">
    <img class='wide_img' src="../LV_client/quick_start/imgs/play.png"/>
    </div>

    <p>&nbsp;</p>

    Go to [LabVIEW Setup](LV_client/quick_start/LabVIEW_client_quick_start.md)

    <p>&nbsp;</p>