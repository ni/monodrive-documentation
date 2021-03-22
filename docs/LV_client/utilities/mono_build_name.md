# mono_build_name.vi

<p class="img_container">
<img class="lg_img" src="../mono_build_name.png"/>
</p>

### Description

Builds the name of a file based on the type of sensor and the custom name provided the user.

For technical support contact us at <b>support@monodrive.io</b>  

### Inputs

- **Configuration:**  Configuration for the sensors. Each sensor has an output
named "Configuration" wire that outputs to this input.
 

- **Name:**  Name appended to the type of sensor. For example, if the type
of sensor is Camera and its name is "Front", the resulting
name will be "Camera_Front.bin".
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Type:**  The type of sensor being configurated is extracted from the
JSON configuration.
 

- **Name:**  The constructed name for the log file is going to be
created based on the type of sensor and the custom name provided by the user.
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
