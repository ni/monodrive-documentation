## mono_read_trajectory.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/monoDrive_lvlib_mono__read__trajectoryc.png" 
width="400"  />
</p>

### Description 
Read a trajectory string and modify the EGO pose with values given by the user on all the frames. Tipically for HIL.

### Inputs

- **Trajectory string in(String):** String with the trajectory information obtained from reading the configuration file.
- **EGO pose(Cluster):** Setting of the EGO vehicle usually set from HIL .

| Type  | Name   |
| ------------ | ------------ |
|1D Array DBL  | angular velocity |
|1D Array DBL | orientation  |
|1D Array DBL | position  |
|1D Array DBL  | velocity |
|DBL | steering_direction |

- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **Trajectory string out:** Trajectory with the new pose for the EGO vehicle in all the frames.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
