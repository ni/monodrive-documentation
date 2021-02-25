# Linux

[Download monoDrive](https://www.monodrive.io/register)

[Simulator & Scenario Editor Prerequisites](../../../Getting_Started)

## Simulator

1. Extract the archive to a common location like `~/monodrive`.
1. If downloading for the first time, you will need to download the license.txt file through your account on [monoDrive.io](https://www.monodrive.io/register), unless one has been provided directly from monoDrive. Copy the attached license.txt file to the extracted location `~/monodrive/VehicleAI/license.txt`.
1. Run the simulator by navigating the `~/monodrive/VehicleAI` directory and running the shell script `VehicleAI.sh -opengl`.

## Scenario Editor

1. Extract the archive to a common location like `~/monodrive`.
1. If downloading for the first time, you will need to download the license.txt file through your account on [monoDrive.io](https://www.monodrive.io/register), unless one has been provided directly from monoDrive. Copy the attached license.txt file to the extracted location `~/monodrive/VehicleAI_Editor/license.txt`.
1. Clone [Unreal Engine: branch 4.25.4](https://www.unrealengine.com/en-US/).
1. Extract the Plugins.zip archive into the 4.25.4 Engine's Plugins directory. e.g. if the UE4.25.4 branch is cloned at `/usr/local/UE_4.25.4`, then extract the archive into `/usr/local/UE_4.25.4/Engine/Plugins`. The resulting directory structure should look as follows:
  <pre>
  /usr/local/UE_4.25.4/Engine/Plugins/monoDrive
    +-- monoDriveRadarSensor
    +-- monoDriveLidarSensor
    +-- ... (other monoDrive plugins)
  </pre>

1. Setup & Generate Visual Studio project files. Open the terminal in the Unreal Engine Folder,  `/usr/local/UE_4.25.4/`, and run these commands.
    <pre>
      $ Setup.sh
      $ GenerateProjectFiles.sh
      $ make
    </pre>

1. Afterwards to launch the Scenario Editor or Simulator:
  <pre>
    ${UE4_ROOT}/Engine/Binaries/Linux/UE4Editor 
    ${SIMULATOR_ROOT}/Simulator/VehicleAI.uproject
  </pre>
    Note that the editor may take a long time to compiling shaders, and may appear to get stuck at 95% or 99%. This is expected and you will need to wait for the compiling to finish. If no error is reported then the system is still compiling shaders, this only happens the first time opening. 

<p>&nbsp;</p>

  Go to [LabVIEW Client Setup](../../LV_client/quick_start/LabVIEW_client_quick_start.md)
  
  Go to [C++ Client Setup](../../cpp_client/cpp_quick_start.md)

  Go to [Python Client Setup](../../python_client/quick_start.md)

<p>&nbsp;</p>