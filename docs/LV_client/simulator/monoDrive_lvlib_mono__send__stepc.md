## mono_send_step.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/simulator/monoDrive_lvlib_mono__send__stepc.png" 
width="400"  />
</p>

### Description 
Send the command to step the simulation for a given amount. Specify the desired amount in the message string.

### Inputs

- **TCP Network connection in (TCP Network connection) :** TCP connection to the server .
- **Command_ID (String) :** A valid command to the server. Typically **REPLAY_ConfigureSensorsCommand_ID**
- **message :** a JSON format text. Used in Replay mode and replya_step mode.
  * Replay: A specific amount (1,2,3, etc...)
  ```{"amount": 1}```
  * Replay_step: One frame of a trajectory. Used for HIL.

- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **TCP Network connection out (TCP Network connection) :** TCP connection to the server .
- **success (Boolean):** **True** if configuration was succesful, **False** otherwise .
- **config response (String):** Response from the server.

- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
