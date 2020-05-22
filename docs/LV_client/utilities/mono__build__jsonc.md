## mono_build_json.vi
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/mono__build__jsonc.png"   />
</p>

### Description 
Builds a JSON message to send via TCP with format "type", "success", "reference", "message".a

### Inputs

- **Message (String):** JSON format text.
- **Command_ID (String) :** A valid command to the server.

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


### Outputs

- **Json Out:** A valid JSON format message to send to the server with the following structure.
```

{
      "type": string,
      "success": bool,
      "reference": int,
      "message": JSON,
}
```
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>