# mono_start_hil.vi

<p class="img_container">
<img class="lg_img" src="../mono_start_hil.png"/>
</p>

### Description

Tool for initialization of a HIL mode program.

For technical support contact us at <b>monodrive.support@ni.com</b> 

### Inputs

- **Trajectory Configuration:**  Path to the Trajectory (replay) file 
 

- **Weather Profile Ref:**  Reference to the Weather Profile array 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs
- **Trajectory Configuration:**  Trajectory (replay) read from the input file

- **Weather Profiles:** Array of the weather profiles obtained by the weather configuration.

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
