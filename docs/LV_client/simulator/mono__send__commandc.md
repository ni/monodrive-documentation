## mono_send_command.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/simulator/mono__send__commandc.png" width="400"  />
</p>

### Description 
Send a command via TCP with format "type", "success", "reference",  "message". This VI is used to send server commands. 

### Inputs

- **TCP Network connection in (TCP Network connection):** TCP connection to the server .
- **JSON_Command :** JSON format with the following structure:
```

{
      "type": string,
      "success": bool,
      "reference": int,
      "message": JSON,
}
```
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **TCP Network connection out (TCP Network connection) :** TCP connection to the simulator.
- **success :** True if the configuration was succesful, otherwise False .
- **response :** Response from the server.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
