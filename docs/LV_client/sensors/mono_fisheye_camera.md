# mono_fisheye_camera.vi

<p class="img_container">
<img class="lg_img" src="../mono_fisheye_camera.png"/>
</p>

### Description

Configures and reads the data stream for a Fisheye Camera sensor and outputs a picture with the specified format in the stream dimensions.

For technical support contact us at **support@monodrive.io** 

### Inputs

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Image:**  Image output with the format and dimensions  specified in
the sensor configuration.
 

- **Raw data:**  Array with pixel values for image. Size is equal to width x
height x 4
 

- **Camera configuration:**  Settings to configure a RGB camera
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
