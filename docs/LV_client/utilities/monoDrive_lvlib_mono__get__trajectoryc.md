## mono_get_trajectory.vi
<p align="center">
<img src="" 
width="400"  />
</p>

### Description 

### Inputs

- **Frames(Cluster):** Cluster with all the frames in a trajectory.
- **Send trajectory number(U32) :** Select a number from 0 to the maximum number of trajectories on the frame cluster. .
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **trajectory (String) :** The frame selected in JSON format.
- **No.of trajectories (U32):** Maximum number of frames in a trajectory .
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
