## mono_get_server_response.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/monoDrive_lvlib_mono__get__server__responsec.png" 
width="400"  />
</p>

### Description
Get the response from the server after sending all the sensors data.

### Inputs

- **TCP Network Connection in (TCP Network connection) :** TCP connection to the server .

- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **Status (Boolean):** True if the server was succesful at sending the data, otherwise False.
- **response (String):** Response from the server..
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
