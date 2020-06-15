# mono_read_trajectory_frame.vi

<p class="img_container">
<img class="lg_img" src="../mono_read_trajectory_frame.png"/>
</p>

### Description

Modify the EGO pose with values given by the user on one frame of the given trajectory file. 
For technical support contact us at support@monodrive.io

### Inputs
- **monoDrive in (Cluster):** See description at [**monoDrive.ctl**](../structures/monoDrive.md). 


- **Frame number:**  Frame number to get from the array of frames
 

- **Ego pose:**  Values to replace the pose of the ego vehicle
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Trajectory Frame String:**  Trajectory frame (JSON) with the replaced pose for the ego
vehicle
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
