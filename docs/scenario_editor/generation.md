# Generating a Simulator Build
You can generate release builds of the monoDrive simulator from the Scenario Editor that
include custom code and assets. Builds can be generated for both Windows and Linux
using the Windows Scenario Editor. Custom code can be added to the simulator under the
`VehicleAI_Editor/Source` folder, or as Unreal Engine 4 plugins by adding the plugins
to `VehicleAI_Editor/Plugins` and modifying the `VehicleAI.uproject` file accordingly.
Custom assets can be imported into the editor using the Unreal Engine asset import 
process, or by adding uassets/blueprints to `VehicleAI_Editor/Content`.

The monoDrive simulator uses CUDA for certain features. Consequently, in order to 
build the standalone simulator from the scenario editor, the CUDA SDK must be installed
on the system. This version of the scenario editor uses CUDA 10.2: 
http://developer.download.nvidia.com/compute/cuda/10.2/Prod/local_installers/cuda_10.2.89_441.22_win10.exe

Additionally, in order to generate the Linux standalone build from Windows, the Linux
CUDA libraries must be added to the Windows CUDA installation. These libraries are included
in `VehicleAI_Editor/ThirdParty/cuda_linux_libraries.zip`. This archive should be extracted to
`C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v10.2`.

Finally, the Windows standalone build requires a few DLLs to be included with the build. 
These DLLs are included in `VehicleAI_Editor/ThirdParty/windows_simulator_dlls.zip`. Extract
the contents of this zip file into the directory of the generated release build.

### Build Steps 
1. To generate the standalone simulator, open the Scenario Editor and go to 
`File -> Package Project -> Build Configuration` and select `Shipping`. Then go to
`File -> Package Project -> Windows -> Windows (64 bit)` to generate the Windows build,
or `File -> Package Project -> Linux -> Linux` to generate the Linux x64 build.

<div class="img_container">
  <img class='wide_img' src="../imgs/build.png"/>
</div>

2. A dialog will prompt you to select a location for generating the build. This folder is
where the final `VehicleAI.exe` and it's related content/files will be created.

3. For the Windows release build, copy the DLLs referenced above into the directory. 
The DLLs should be in the same directory as `VehicleAI.exe`.