## mono_draw_spawn_positionsc.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/utilities/mono__draw__spawn__positionsc.png" 
width="400"  />
</p>

### Description 
Draw the possible spawn positions for the EGO vehicle on the map as dots. Adds the string "Camera" for the position where the camera is
pointing (only on Editor mode).

### Inputs
- **monoDrive in (Cluster):** See description at monoDrive.ctl.
- **spawn_positions(DBL Array):** The x,y,z location for the start locations defined in the map.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **monoDrive out (Cluster): :**  .
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
