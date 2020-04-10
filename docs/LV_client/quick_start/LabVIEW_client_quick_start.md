# LabVIEW Client Quick Start

## Prerequisites 

1. Download LabVIEW 2019 (64 bit) 

1. Install Unreal Engine **4.23** from [here](https://www.unrealengine.com/en-US/).

    - In Epic Games Launcher, and select Marketplace >> Browse. Search for "VehicleSim Dynamics" and download this free plugin. 
    - Alternatively, from Epic Games website, sign-in and download the plugin [here](https://www.unrealengine.com/marketplace/en-US/product/carsim-vehicle-dynamics).

1. Download NVIDIA CUDA Toolkit 10.2 from [here](https://developer.nvidia.com/cuda-downloads?target_os=Windows&target_arch=x86_64).
    - Choose OS/ Version and how to install
    - Example:
         - Operating System: Windows
         - Architecture: x84_64
         - Version: 10
         - Installer Type: exe(local)

1. Download the monoDrive Simulator or monoDrive Simulator Editor from [here](https://www.monodrive.io/register).

1. **Open the VI Package Manger as Administrator** and install the monoDrive Client.

<div class="img_container">
    <img class='lg_img' src="https://github.com/monoDriveIO/documentation/raw/master/docs/LV_client/quick_start_img/prereq.png"/>
</div>

<p>&nbsp;</p>


## Run the VehicleAI Simulator

1. Go to your VehicleAI directory and find VehicleAI.exe

2. Double-click on VehicleAI.exe

3. Move to one side of your screen

<div class="img_container">
    <img class='lg_img' src="https://github.com/monoDriveIO/documentation/raw/master/docs/LV_client/quick_start_img/runVehicleAI.png"/>
</div>

<p>&nbsp;</p>


## Run the VehicleAI Simulator Editor
Quick start instructions and details on how to generate project files in the VehicleAI Simulator Editor can be found in [scenario editor](../../Scenario_editor.md).

1. Download Visual Studio from [here](https://visualstudio.microsoft.com/).

    - Make sure the following options enabled:
        - Workloads: Game development with C++
            - Options: C++ Profiling Tools

    *For more information, check [here](https://docs.unrealengine.com/en-US/Programming/Development/VisualStudioSetup/index.html)*

2. Open VehicleAI_Editor zip file and extract all files

    <div class="img_container">
    <img class='lg_img' src="https://github.com/monoDriveIO/documentation/raw/master/docs/LV_client/quick_start_img/sensor_editor_extract.png"/>
    </div>

3. Go to your VehicleAI_Editor directory and find VehicleAI.uproject

4. Double-click on VehicleAI.uproject

<p>&nbsp;</p>

## Open the LabVIEW Client

1. Navigate to the monoDriveClient folder, you can find it on your NI Examples, typically on:

    *C:\Program Files\National Instruments\LabVIEW 2019\examples\monoDrive\monoDriveClient* 

    You can also find the examples on the NI example Finder.

    Navigate to Help -> Find Examples...

    Select Directory Structure

    Expand the monoDrive folder


    <div class="img_container">
    <img class='lg_img' src="https://github.com/monoDriveIO/documentation/raw/master/docs/LV_client/quick_start_img/find_examples.png"/>
    </div>

    <div class="img_container">
    <img class='lg_img' src="https://github.com/monoDriveIO/documentation/raw/master/docs/LV_client/quick_start_img/find_examples2.png"/>
    </div>


    You can also find the examples on the NI example Finder.

2. The client provides the following examples:

    - mono_closed_loop_example.vi
    - mono_replay_example.vi
    - mono_replay_controls_example.vi
    - mono_multi_vehicle_pose_update_example.vi
    - mono_hil_example.vi
    - mono_lane_follower_cpp_dll.vi
    - mono_radar_with_kalman_filter_cpp_dll.vi

3. Double-click on the VI to open the example.


<p>&nbsp;</p>


For technical support contact us at <b>support@monodrive.io</b>
<p>&nbsp;</p>
