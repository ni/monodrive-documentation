# mono_generate_frame.vi

<p class="img_container">
<img class="lg_img" src="../mono_generate_frame.png"/>
</p>

### Description

Obtains a frame from the array of frames and generates the JSON text to send to the simulator.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **Frames:**  Trajectory frames on array format
 

- **Frame number:**  Frame number to obtain from the *Frames* array
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Frame:**  JSON string representation for the frame obtained from the
*Frames* array and the *Frame number*
 

- **Done:**  True when all the frames have been sent
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
