## mono_init.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/simulator/monoDrive_lvlib_mono__initc.png" 
width="400"  />
</p>

### Description 
Validate the license statud for the monoDrive Software (Windows only).

### Inputs
- **Simulator configuration (String):** A JSON string with the properties for simulator connection, material propertities, etc.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **monoDrive out (Cluster):** See description at monoDrive.ctl.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
