# mono_camera.vi

<p class="img_container">
<img class="lg_img" src="../mono_camera.png"/>
</p>

### Description

Reads the stream data for a RGB camera sensor and returns a picture with the format specified in the stream dimensions.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Image:**  Image output with the format and dimensions  specified in
the sensor configuration.
 

- **Raw data:**  Array with pixel values for image. Size is equal to w*h*4
 

- **pixel:**  Pixel value
 

- **Camera configuration:**  Settings to configure sensor.
 

- **Annotation Data:**  Annotation data for all objects classified on the image
 

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
