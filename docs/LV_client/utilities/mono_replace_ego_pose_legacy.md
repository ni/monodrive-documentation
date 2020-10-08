# mono_replace_ego_pose_legacy.vi

<p class="img_container">
<img class="lg_img" src="../mono_replace_ego_pose_legacy.png"/>
</p>

### Description

Replace the pose for the ego vehicle on a specific frame and outputs the JSON corresponding to the frame with the new settings. 
**This VI processes frames on a format prior to the 1.11 release**

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **Ego pose:**  Values to replace the pose of the ego vehicle on a specific frame.
 

- **Frame number:** Frame number to obtain from the list of frames

- **Frames:**  List with all the frames from the trajectory file

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Frame:**  Frame on JSON format with the new pose for the ego vehicle

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
