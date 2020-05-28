# mono_depth_camera.vi

<p class="img_container">
<img class="lg_img" src="../mono_depth_camera.png"/>
</p>

### Description

Configures and reads the stream data for a depth camera sensor and outputs a picture of the size specified on the Camera configuration.

For technical support contact us at <b>support@monodrive.io</b>
 

### Inputs

- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Raw data:**  Array with pixel values for image. Size is equal to w*h
 

- **pixel:**   

- **Camera configuration:**  Settings to configure sensor.
 

- **Image:**  Image output with the format and dimensions  specified in
the sensor configuration.
 

- **Annotation:**  Annotation data for all objects classified on the image
 

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
