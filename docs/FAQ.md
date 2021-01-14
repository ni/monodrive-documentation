# F.A.Q.

## Installation

 - **Build Errors**
     - Ensure that you have followed all the instructions in [getting started](Getting_Started.md). 
     - Cuda SDK is a requirement for the scenario editor, see [prerequisites](Getting_Started.md).
     - Ensure you have Visual Studio 2019, see [prerequisites](Getting_Started.md).

 - **Long Loading Time**
    - monoDrive's Scenario Editor has high quality graphics that will take some time to compile when opening for the first time. 

 - **License does not work on new machine**
    - Each license is for one machine's use only. Please contact us through email at support@monodrive.io or through a communication channel already established, so we can help you resolve this issue. 

 - **License is not found**
    - Ensure that you have copied a valid `license.txt` file into the correct folder, and that the name of the file is `license.txt`. It should be next to the VehicleAI.exe or the VehicleAI.uproject

## Debbuging

 - **Logs**   
 monoDrive saves a log for Scenario Editor and the simulator on the following locations:
     - Scenario Editor   
         - `Installation Folder\VehicleAI_Editor\Saved\Logs\VehicleAI.log`
     - Simulator   
        - By default logging is disable. To enable logging set the flag `bUseLoggingInShipping` located on the file `Simulator/Source/VehicleAI.Target.cs`. The generated log will be saved on `C:\Users\UserName\AppData\Local\GameName`   
        - If a crash occurr, the log will be saved in the following location: `C:\Users\{user}\AppData\Local\GameName\Saved\Crashes`
