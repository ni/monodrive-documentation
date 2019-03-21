## mono_get_frames.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/blob/lv_client_docs/WikiPhotos/LV_client/utilities/monoDrive_lvlib_mono__get__framesc.png" 
width="400"  />
</p>

### Description 
Modify the information of the EGO vehicle in a given trajectory with external values.

### Inputs

- **Vehicle_name (String):** Vehicle's name obtained from the JSON object attributes.
- **JSON Object (JSON Object):** JSON Object (one trajectory) .
- **angular_velocity (Array of DBL) :** Desired value for EGO's angular velocity .
- **orientation (Array of DBL):** Desired value for EGO's EGO's orientation  .
- **position (Array of DBL):** Desired value for EGO's position .
- **steering_direction (DBL):** Desired value for EGO's steering_direction .
- **velocity (Array of DBL):** Desired value for EGO's velocity .
- **Frame_last (String):** To use with shift register, last value obtained. 
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **Frame (String):** New value for the frame after replacing angular_velocity, orientation, position, steering_direction and velocity of the EGO vehicle with the desired values..
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
