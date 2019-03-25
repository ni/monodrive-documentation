## mono_send_trajectory.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/blob/master/WikiPhotos/LV_client/vehicle/monoDrive_lvlib_mono__send__trajectoryc.png" 
width="400"  />
</p>

### Description 
Configure the trajectory EGO and other cars in the scene will follow  by given a path to JSON file.

### Inputs

- **TCP Network Connection in (TCP Network connection) :** TCP connection to the server .
- **Trajectory Config (Path):** Path or name of the file with the Sensor configuration in JSON format.
- **Command_ID (String) :** A valid command to the server. Typically **REPLAY_ConfigureTrajectoryCommand_ID**

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

- **Running mode (Enum)** 
    * 0 - Closed_loop 
    * 1 - Replay
    * 2 - Replay_step 
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **config response (String):** Response from the server.
- **TCP Network Connection Out (TCP Network connection):** TCP connection to the server.
- **AllTrajectories (Boolean):** JSON object with the trajectories read from the configuration file .
- **trajectories config (Int) :** Number of trajectories in the configuration file.
- **number of trajectories (Int) :** Number of trajectories in the configuration file.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
