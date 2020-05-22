## mono_read_trajectory.vi
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/mono__read__trajectoryc.png"   />
</p>

### Description 
Read a trajectory string and modify the EGO pose with values given by the user on all the frames. Tipically for HIL.

### Inputs

- **monoDrive in (Cluster):** See description at **monoDrive.ctl**.
- **trajectory_number(U32):** Index of the frame chosen from the trajectory.
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

<p>&nbsp;</p>