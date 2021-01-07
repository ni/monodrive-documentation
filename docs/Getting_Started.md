# Prerequisites 

### Scenario Editor

1. Ensure that [Visual Studio 2019](https://visualstudio.microsoft.com/downloads/) is installed to generate the project files.

1. Install [Cuda Toolkit 10.2](https://developer.nvidia.com/cuda-10.2-download-archive)
    - During installation under "Options", "Express" will update or download the NVIDIA driver for this version of Cuda Toolkit. 
    - (Optional) If you have already updated to the most recent NVIDIA display drivers, you may want to customize this install under "Options", then "Custom". The next window will display installation options including "Driver Components", where you may remove checks to keep the driver version you currently have.

<div class ='img_container'>

<div class="img_container">
    <img class='md_img' src="../imgs/cuda_options.png"/>
</div>

<div class="img_container">
    <img class='md_img' src="../imgs/cuda_options2.png"/>
</div>
  
</div>

### Simulator

1. Ensure you have the latest NVIDIA drivers for your [CUDA-enabled](https://developer.nvidia.com/cuda-gpus) graphics card.

	<div class="img_container">
    <img class='sm_img' src="../imgs/nvidia_driver2.png"/>
	<img class='semiwide_img' src="../imgs/nvidia_driver1.png"/>
    </div>

    **NOTE**
    Even if not prompted by the system, a restart is recommended to ensure the new drivers are correctly detected prior to running any simulations.

    In case the driver cannot be updated using the above method, please visit https://www.nvidia.com/Download/index.aspx to download the [latest driver](https://www.nvidia.com/Download/index.aspx).

<p>&nbsp;</p>

<hr width="70%"/>

<p>&nbsp;</p>

[Go to Windows Setup](../monoDrive_home/getting_started/Windows)

[Go to Linux Setup](../monoDrive_home/getting_started/Linux)

<p>&nbsp;</p>