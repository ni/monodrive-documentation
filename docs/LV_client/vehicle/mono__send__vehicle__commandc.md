## mono_send_vehicle_command.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/lv_client_docs/WikiPhotos/LV_client/vehicle/mono__send__vehicle__commandc.png?raw=true" 
width="400"  />
</p>

### Description 
Sends a command to the monoDrive Simulator to control the movement of ego vehicle.

### Inputs

- **TCP Connecton in (TCP Network connection):** TCP connection to the server .
- **forward_amount:** a value between -1.0 and 1.0 that determines the amount of throttle to apply.
- **right_amount :** a value between -1.0 and 1.0 that determines the steering position (-1 is left, +1 is right).
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **TCP Connecton out (TCP Network connection):** TCP connection to the server .
- **success (Boolean):** **True** if configuration was succesful, **False** otherwise .
- **vehicle command response (String):** Response from the server.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
