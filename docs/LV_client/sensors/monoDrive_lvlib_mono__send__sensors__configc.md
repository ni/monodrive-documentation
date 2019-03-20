## mono_send_sensors_config .vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/blob/lv_client_docs/WikiPhotos/LV_client/sensors/monoDrive_lvlib_mono__send__sensors__configc.png?raw=true" 
width="400"  />
</p>

### Inputs

- **TCP Connection in (TCP Network connection) :** TCP connection to the server .
- **Sensor Config (Path):** Path or name of the file with the Sensor configuration in JSON format.
- **Command_ID (String) :** A valid command to the server.

| Name  |
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

**error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **config response (String):** Response from the server.
- **TCP Connection Out (TCP Network connection):** TCP connection to the server.
- **success (Boolean):** **True** if message succesful, **False** if message failed .
- **ports (Array of int):** Array with all the number ports for sensors specified in the Sensor config file.
- **Sensors(Array of String):** Array with the name type of all the sensors specified in the Sensor config file.
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
