# Scenario Editor

The monoDrive Scene Editor extends the monoDrive Simulator and allows the user to control any of the thousands of features available in the monoDrive simulation environments. With the Scene Editor, users are able to generate custom scenarios leveraging the monoDrive lane and vehicle AIs. Custom simulator maps, vehicle parameters, and driving behaviors can be generated in the Scene Editor all of which can be used to test AV algorithms using any one of the monoDrive clients.


## Getting Started

1. Install Unreal Engine **4.23** from [here](https://www.unrealengine.com/en-US/).

    <p>&nbsp;</p>

1. In Epic Games Launcher, and select Marketplace >> Browse. Search for "VehicleSim Dynamics" and download this free plugin. 
    - Alternatively, from Epic Games website, sign-in and download the plugin [here](https://www.unrealengine.com/marketplace/en-US/product/carsim-vehicle-dynamics).

    <p>&nbsp;</p>

1. Ensure you have the latest NVIDIA drivers for your [CUDA-enabled](https://developer.nvidia.com/cuda-gpus) graphics card. 

    <div class="img_container">
    <img class='sm_img' src="../nvidia_driver2.png"/>
    </div>

    <div class="img_container">
    <img class='wide_img' src="../nvidia_driver1.png"/>
    </div>
    <p>&nbsp;</p>

    **NOTE**
    Even if not prompted by the system, a restart is recommended to ensure the new drivers are correctly detected prior to running any simulations.

    **NOTE**
    In case the driver cannot be updated using the above method, please download the [latest driver here](http://www.nvidia.com/Download/index.aspx).

    <p>&nbsp;</p>

1. Download Visual Studio from [here](https://visualstudio.microsoft.com/).

    - Make sure the following options enabled:

         - Workloads: Game development with C++

            - Options: C++ Profiling Tools

    *For more information, check [here](https://docs.unrealengine.com/en-US/Programming/Development/VisualStudioSetup/index.html)*

    <p>&nbsp;</p>


1. Download the monoDrive Simulator Editor [here](https://www.monodrive.io/register) and extract all files. 

    <p>&nbsp;</p>

1. Go to your VehicleAI_Editor directory and find VehicleAI.uproject

    **NOTE** 
    You can run the simulator by opening VehicleAI.uproject

<p>&nbsp;</p>

## Generate Project Files

1. Generate Visual Studio project files by right clicking on VehicleAI.uproject in the VehicleAI directory. 

    <div class="img_container">
    <img class='lg_img' src="../LV_client/quick_start/imgs/generate_project_files.png"/>
    </div>

2. Double-click on VehicleAI.sIn to open the Simulator.

    <div class="img_container">
    <img class='lg_img' src="../LV_client/quick_start/imgs/vehicle-sIn.png"/>
    </div>

## Run

1. Play Simulator

    <div class="img_container">
    <img class='wide_img' src="../LV_client/quick_start/imgs/play.png"/>
    </div>

    <p>&nbsp;</p>

    Go to [LabVIEW Setup](LV_client/quick_start/LabVIEW_client_quick_start.md)

    <p>&nbsp;</p>