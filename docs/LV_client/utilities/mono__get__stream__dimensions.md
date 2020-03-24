## mono_get_stream_dimensions.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/utilities/mono__get__stream__dimensionsc.png" width="400"  />
</p>

### Description 
Get the dimensions from the camera configuration.

### Inputs
- **Sensor configuration text (String):** JSON formated text that specifies the camera configuration.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **x (I32):** The width of the camera output.
- **y (I32):** The height of the camera output.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
