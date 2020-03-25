# mono_kf_updatec.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/shared_libraries/mono__kf__updatec.png" 
width="400"  />
</p>

### Description 
Call the function `Update` from the DLL.

### Inputs
- **aoa:** Aoa value from the radar output.
- **dt:** Delta time between samples.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **KF_dll_path :** Path to the Release or Debug DLL. .
- **aoa_kf :** Aoa value from the Kalman Filter.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
