## mono_initializationc.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/blob/lv_client_docs/WikiPhotos/LV_client/utilities/monoDrive_lvlib_mono__initializationc.png?raw=true" 
width="400"  />
</p>

### Description 
Takes care of initilization given a path to the sensor, simulator and trajectory configuration files. Checks for licensing.

### Inputs

- **Simulator Config (Path):** Path or name of the file with the Simulator configuration in JSON format.
- **Sensor Config (Path):** Path or name of the file with the Sensor configuration in JSON format.
- **Trajectory Config (Path):** Path or name of the file with the Trajectory configuration in JSON format.
SimulatorConfig_ID
- **SensorsCommand_ID:** A valid command to the server..
- **TrajectoryCommand_ID:** A valid command to the server..
- **Control Port (int):** Port to write and read messages from the simualor, typically **8999**.
- **Simulator IP (string):** IP of the simulator, typically **127.0.0.1**.
- **Running mode (Enum)** 
    * 0 - Closed_loop 
    * 1 - Replay
    * 2 - Replay_step 
- **Base_Path:** $(User Home directory)/monoDriveConfig. The directory gets created when installing the package.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- ** :**  .
- ** :**  .
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
