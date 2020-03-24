## calculate_orientation_xyz.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/mono__calculate_orientation_xyz.png" width="400"  />
</p>

### Description 
Converts orientation Yaw, Pitch and roll to X,Y,Z,W .

### Inputs

- **Orientation (Cluster):** Cluster with orientation Yaw,Pitch and Roll in degrees.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **pose(Array DBL):**  Orientation on X,Y,Z,W.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
