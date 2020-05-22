## mono_start_point.vi
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/utilities/mono__start__pointc.png" 
  />
</p>

### Description 
Get map information, starting points and draw on the map.

### Inputs
- **monoDrive in (Cluster):** See description at monoDrive.ctl.
- **Vehicle Start Position(I32):** Selected start point.
- **message(String):** message to query the map to the server in geojson or array format.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **monoDrive in (Cluster):** See description at monoDrive.ctl.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>