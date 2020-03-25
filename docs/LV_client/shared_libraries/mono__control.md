## mono_control.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/shared_libraries/mono__controlc.png" 
width="400"  />
</p>

### Description 
Given a State sensor sample (one frame), initialized the `LaneFollower` DLL and calculate the steering angle to mantain the car on the lane
on the left.

### Inputs
- **State Sensor sample(String):** The output of the state sensor. Corresponding to one frame of the simulation.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **Steering(DBL):** Steering value calculated by DLL to maintain the car on the right lane.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
