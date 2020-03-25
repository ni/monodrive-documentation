## mono_control_vehicle.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/shared_libraries/mono__control__vehiclec.png" 
width="400"  />
</p>

### Description 
Call the `control_vehicle`. Given json frame, calculates the steering value.

### Inputs
- **DLL path in(Path):** Path to the Release or Debug DLL .
- **frame(String):** A JSON string from the state sensor correspoding to one frame.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **DLL path out(Path):** Path to the Release or Debug DLL.
- **return(DBL):** The return value from the DLL function (steering value).
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
