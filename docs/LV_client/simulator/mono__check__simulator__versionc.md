## mono_check_simulator_version.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/simulator/mono__check__simulator__versionc.png" 
width="400"  />
</p>

### Description 
Check compatibility between the simulator's version and the client's version.

### Inputs

- **message_JSON(String):** The message section of the server response in JSON format.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **Compatible(Boolean):** True if the major version of the simulator is the same version specified in the client.
- **version_number_array(Array of int) :** An array of size 3 with the version number.
- **error out (Error Cluster):** Can accept error information wired from VIs 
