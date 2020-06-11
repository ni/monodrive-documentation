# mono_build_json.vi

<p class="img_container">
<img class="lg_img" src="../mono_build_json.png"/>
</p>

### Description

Builds a JSON message to send via TCP with format:   
```json
    {
        "type": string,
        "success": bool,
        "reference": int,
        "message": JSON
    }
```

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **Message:**  Valid message for the **Command_ID** sent
 

- **Command_ID:**  Valid monoDrive API command to send to the server:    

| Valid commands  |
|  | 
|Status_ID   |
|SimulatorConfig_ID |
|EgoVehicleConfig_ID |
|StreamData_ID   |
|WaypointUpdate_ID    |
|ScenarioInit_ID  |
|ScenarioConfig_ID   |
|MapCommand_ID  |
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
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **JSON Out:**  Valid JSON formatted string to send to the server with the
following structure:    

```json
    {
        "type": string,
        "success": bool,
        "reference": int,
        "message": JSON
    }
```
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
