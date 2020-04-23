# Getting Started

## Product Information

**Simulator**

The monoDrive Simulator provides users with the ability to test AV algorithms in one of several pre-made monoDrive Simulator levels. The Open Source monoDrive clients provide an API and examples for connecting to the Simulator for simulating traffic scenarios and replaying monoDrive trajectory files. The clients allow users to configure and test any number of configurations of monoDrive high-fidelity sensors.

**Scene Editor**

The monoDrive Scene Editor extends the monoDrive Simulator and allows the user to control any of the thousands of features available in the monoDrive simulation environments. With the Scene Editor, users are able to generate custom scenarios leveraging the monoDrive lane and vehicle AIs. Custom simulator maps, vehicle parameters, and driving behaviors can be generated in the Scene Editor all of which can be used to test AV algorithms using any one of the monoDrive clients.

<p>&nbsp;</p>


## Prerequisites 

1. Install Unreal Engine **4.23** from [here](https://www.unrealengine.com/en-US/).


1. In Epic Games Launcher, and select Marketplace >> Browse. Search for "VehicleSim Dynamics" and download this free plugin. 
    - Alternatively, from Epic Games website, sign-in and download the plugin [here](https://www.unrealengine.com/marketplace/en-US/product/carsim-vehicle-dynamics).


1. Ensure you have the latest NVIDIA drivers for your [CUDA-enabled](https://developer.nvidia.com/cuda-gpus) graphics card.

	<div class="img_container">
    <img class='sm_img' src="../nvidia_driver2.png"/>
	<img class='semiwide_img' src="../nvidia_driver1.png"/>
    </div>

    **NOTE**
    Even if not prompted by the system, a restart is recommended to ensure the new drivers are correctly detected prior to running any simulations.

    **NOTE**
    In case the driver cannot be updated using the above method, please visit http://www.nvidia.com/Download/index.aspx to download the [latest driver](https://www.nvidia.comnvidia.com).
    

<p>&nbsp;</p>
<hr width="70%"/>
<p>&nbsp;</p>


## Simulator 

1. Download the monoDrive Simulator from [here](https://www.monodrive.io/register).

<p>&nbsp;</p>

## Scene Editor

1. Download the monoDrive Simulator Editor from [here](https://www.monodrive.io/register) and extract all files. 

1. Download Visual Studio from [here](https://visualstudio.microsoft.com/).

    - Make sure the following options enabled:

         - Workloads: Game development with C++

            - Options: C++ Profiling Tools

    *For more information, check [here](https://docs.unrealengine.com/en-US/Programming/Development/VisualStudioSetup/index.html)*


1. Go to your VehicleAI_Editor directory and find VehicleAI.uproject

    **NOTE** 
    You can run the simulator by opening VehicleAI.uproject

<p>&nbsp;</p>

### Generate Project Files in Scenario Editor

1. Generate Visual Studio project files by right clicking on VehicleAI.uproject in the VehicleAI directory. 

    <div class="img_container">
    <img class='lg_img' src="../LV_client/quick_start/imgs/generate_project_files.png"/>
    </div>

2. Double-click on VehicleAI.sIn to open the Simulator.

    <div class="img_container">
    <img class='lg_img' src="../LV_client/quick_start/imgs/vehicle-sIn.png"/>
    </div>

### Run Scenario Editor

1. Play Simulator

    <div class="img_container">
    <img class='wide_img' src="../LV_client/quick_start/imgs/play.png"/>
    </div>

    <p>&nbsp;</p>

    Go to [LabVIEW Setup](LV_client/quick_start/LabVIEW_client_quick_start.md)

    <p>&nbsp;</p>