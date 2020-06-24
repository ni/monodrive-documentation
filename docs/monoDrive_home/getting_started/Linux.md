# Linux

## Simulator

  1. Extract the archive to a common location like "~/monodrive".
  1. If downloading for the first time, you will receive an email with a license.txt file attachment. Copy the attached license.txt file to the extracted location "~/monodrive/VehicleAI/license.txt"
  1. Run the simulator by navigating the "~/monodrive/VehicleAI" directory and running the shell script "VehicleAI.sh -opengl"

## Scenario Editor

  1. The download will provide a shell script that will download and build Unreal Engine, OpenCV and the monoDrive Scenario Editor. To install just run `bash ./VehicleAI_Editor_installer_v1.11.sh`. You will be walked through the installation process by the script.
  1. If downloading for the first time, you will receive an email with a license.txt file attachment. Copy the attached license.txt file to "/usr/local/monodrive/VehicleAI_Editor/license.txt"
  1. The install script will put an alias in your .bashrc that allows you to start the simulator from a terminal with "monodrive_editor"


## Scenario Editor: Generating Project Files

Generation of project files is not a requirement to run the Simulator, but it enables users to make changes to the open source from monoDrive as well as run the debugger.

1. Download [Visual Studio Code](https://code.visualstudio.com/).

1. Generate Visual Studio project files

    `/usr/local/monodrive/UnrealEngine/GenerateProjectFiles.sh -project="/usr/local/monodrive/VehicleAI_Editor/Simulator/VehicleAI.uproject" -game -engine -makefiles`

1. After the file are generated, there will be a Visual Studio Code Workspace file in 

    `/usr/local/monodrive/VehicleAI_Editor/VehicleAI.workspace`

    The Workspace file can also be opened by starting Visual Studio Code and selecting "File -> Open Workspace..."

<p>&nbsp;</p>