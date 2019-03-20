## mono_send_simulator_config.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/blob/lv_client_docs/WikiPhotos/LV_client/simulator/monoDrive_lvlib_mono__send__simulator__configc.png?raw=true" 
width="400"  />
</p>

### Description 
Configures simulator given a path to a  JSON file. The file will configure server ip, port, etc.

### Inputs

- **TCP Connecton in (TCP Network connection):** TCP connection to the server .
- **Simulator Config (Path):** Path or name of the file with the Simulator configuration in JSON format.
- **Command_ID (String) :** A valid command to the server. Typically **SimulatorConfig_ID**

| Valid commands  |
| ------------ | 
|Status_ID   |
|SimulatorConfig_ID |
|EgoVehicleConfig_ID |
|StreamData_ID   |
|WaypointUpdate_ID    |
|ScenarioInit_ID  |
|ScenarioConfig_ID   |
|MapCommand_ID  |
|EgoControl_ID  | 
|UpdateActorCommand_ID  | 
|StartAllSensorsCommand_ID   | 
|StopAllSensorsCommand_ID   | 
|DetachSensorCommand_ID   | 
|AttachSensorCommand_ID   |
|SpawnActorCommand_ID   |
|REPLAY_ConfigureSensorsCommand_ID  |
|REPLAY_StepSimulationCommand_ID  |
|REPLAY_ConfigureTrajectoryCommand_ID  |
|REPLAY_StateStepSimulationCommand_ID   | 

- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
- **Running mode (Enum)** 
    * 0 - Closed_loop 
    * 1 - Replay
    * 2 - Replay_step 


### Outputs

- **TCP Connecton in (TCP Network connection):** TCP connection to the server .
- **success (Boolean):** **True** if configuration was succesful, **False** otherwise .
- **success (Boolean):** Response from the server.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
