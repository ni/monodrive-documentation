## .vi
<p align="center">
<img src="" 
width="400"  />
</p>

### Description 
Send a command via TCP with format "type", "success", "reference",  "message"

### Inputs

- **TCP Connecton in (TCP Network connection):** TCP connection to the server .
- **JSON Command :** JSON format with the following structure:
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

- **TCP Connecton out (TCP Network connection) :** TCP connection to the simulator.
- **success :** True if the configuration was succesful, otherwise False .
- **response :** Response from the server.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
