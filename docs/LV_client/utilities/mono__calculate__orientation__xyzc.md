## mono_calculate_orientation_xyzc.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/utilities/mono__calculate__orientation__xyzc.png" width="400"/>
</p>

## Description 
Converts orientation Yaw, Pitch and roll to X,Y,Z,W .

### Inputs

- **Orientation (Cluster):** Cluster with orientation Yaw,Pitch and Roll in degrees.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **pose(Array DBL):**  Orientation on X,Y,Z,W.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
