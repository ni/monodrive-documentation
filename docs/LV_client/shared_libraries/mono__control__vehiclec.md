## mono_control_vehicle.vi
<p class="img_container">

<img class="lg_img" src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/shared_libraries/mono__control__vehiclec.png" />
</p>

### Description 
Given a frame from the State sensor, calculates the correct steering value to maintain the vehicle on the second lane.

For technical support contact us at support@monodrive.io

### Inputs
- **DLL path in(Path):** Path to the Release or Debug DLL .
- **frame(String):** A JSON string from the state sensor corresponding to one frame.
- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **DLL path out(Path):** Path to the Release or Debug DLL.
- **return(DBL):** The return value from the DLL function (steering value).
- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>