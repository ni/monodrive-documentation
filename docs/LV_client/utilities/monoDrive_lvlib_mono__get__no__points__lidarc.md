## mono_get_no_points_lidarc.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/monoDrive_lvlib_mono__get__no__points__lidarc.png" 
width="400"  />
</p>

### Description 
Gets the number of points on a data packet depending on the horizontal resolution.

### Inputs

- **horizontal_resolution (DBL):** Angular resolution.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **No.points (I32):** Number of points on a packet data depending on the horizontal resolution.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
