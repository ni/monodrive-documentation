# Simulator

<!-- [Download monoDrive Simulator](https://www.monodrive.io/register) -->

## Prerequisites

1. Ensure you have the latest NVIDIA drivers for your [CUDA-enabled](https://developer.nvidia.com/cuda-gpus) graphics card.

	<div class="img_container">
    <img class='sm_img' src="../../../imgs/nvidia_driver2.png"/>
	<img class='semiwide_img' src="../../../imgs/nvidia_driver1.png"/>
    </div>

    Even if not prompted by the system, a restart is recommended to ensure the new drivers are correctly detected prior to running any simulations.

    In case the driver cannot be updated using the above method, please visit [NVIDIA Download Page](https://www.nvidia.com/Download/index.aspx) to download the latest driver.

## Windows Simulator

1. Extract the archive to a common location like `C:/monodrive`.
1. If downloading for the first time, you will need to download the license.txt file through your account on [monoDrive.io](https://www.monodrive.io/register), unless one has been provided directly from monoDrive. Copy the attached license.txt file to the extracted location `C:/monodrive/VehicleAI/license.txt`.
1. Run the simulator by launching `C:/monodrive/VehicleAI/VehicleAI.exe`.

## Linux Simulator

1. Extract the archive to a common location like `~/monodrive`.
1. If downloading for the first time, you will need to download the license.txt file through your account on [monoDrive.io](https://www.monodrive.io/register), unless one has been provided directly from monoDrive. Copy the attached license.txt file to the extracted location `~/monodrive/VehicleAI/license.txt`.
1. Run the simulator by navigating the `~/monodrive/VehicleAI` directory and running the shell script `VehicleAI.sh`.

## Open Source Clients

Working with the monoDrive Simulator can be done directly through TCP/IP communications with any application that conforms to the monoDrive API. However, monoDrive provides several reference implementations of clients in C++, LabVIEW, and Python. The monoDrive Clients are open source and available to all users of the monoDrive Simulator and Scenario Editor.

 - Go to [C++ Client Setup](../../cpp_client/cpp_quick_start.md)

 - Go to [LabVIEW Client Setup](../../LV_client/quick_start/LabVIEW_client_quick_start.md)

 - Go to [Python Client Setup](../../python_client/quick_start.md)
    