## mono_get_start_points.vi
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/utilities/mono__get__start__pointsc.png" 
  />
</p>

### Description 
Send the GetStartPoints command to the server to get the location of the starting position on a map.

### Inputs
- **monoDrive in (Cluster):** See description at monoDrive.ctl.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **monoDrive out (Cluster):** See description at monoDrive.ctl.
- **data(Cluster):** A cluster with three 1D arrays for `locations`, `rotations` and `type` .
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>