## mono__init.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/blob/lv_client_docs/WikiPhotos/LV_client/simulator/monoDrive_lvlib_mono__initc.png" 
width="400"  />
</p>

### Description 
Module to initialize the monoDrive simulator. This module must be run before other simulator configuration/vehicle commands are run.

### Inputs

- **Simulator IP (string):** IP of the simulator, typically **127.0.0.1**.
- **Control Port (int):** Port to write and read messages from the simualor, typically **8999**.
- **License Status (Reference):** Reference to the license status indicator.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **response :** Response from the server.
- **Success :** True if the configuration was succesful, otherwise False .
- **connection ID out (TCP Network connection) :** TCP connection to the simulator .
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
