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
1. Clone [Unreal Engine: branch 4.25.6](https://www.unrealengine.com/en-US/).
1. Extract the Plugins.zip archive into the 4.25.6 Engine's Plugins directory. e.g. if the UE4.25.6 branch is cloned at `/usr/local/UE_4.25.6`, then extract the archive into `/usr/local/UE_4.25.6/Engine/Plugins`. The resulting directory structure should look as follows:
  <pre>
  /usr/local/UE_4.25.6/Engine/Plugins/monoDrive
    +-- monoDriveRadarSensor
    +-- monoDriveLidarSensor
    +-- ... (other monoDrive plugins)
  </pre>
5. Open VehicleAI.uproject. Note that the first time you open the project, UE4 will prompt you about missing modules that need to be rebuilt (VehicleAI and VehicleAIEditor). Select "Yes" to build the modules, then the project will open.


## Scenario Editor: Generating Project Files (Optional)

Generation of project files is not a requirement to run the Simulator, but it enables users to make changes to the open source from monoDrive as well as run the debugger.

1. Download [Visual Studio Code](https://code.visualstudio.com/).

1. Generate Visual Studio project files

    `/usr/local/monodrive/UnrealEngine/GenerateProjectFiles.sh -project="/usr/local/monodrive/VehicleAI_Editor/Simulator/VehicleAI.uproject" -game -engine -makefiles`

1. After the file are generated, there will be a Visual Studio Code Workspace file in 

    `/usr/local/monodrive/VehicleAI_Editor/VehicleAI.workspace`

    The Workspace file can also be opened by starting Visual Studio Code and selecting "File -> Open Workspace..."

<p>&nbsp;</p>

  Go to [LabVIEW Client Setup](../../LV_client/quick_start/LabVIEW_client_quick_start.md)
  
  Go to [C++ Client Setup](../../cpp_client/cpp_quick_start.md)

  Go to [Python Client Setup](../../python_client/quick_start.md)

<p>&nbsp;</p>