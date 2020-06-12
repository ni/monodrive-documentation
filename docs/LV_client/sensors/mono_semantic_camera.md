# mono_semantic_camera.vi

<p class="img_container">
<img class="lg_img" src="../mono_semantic_camera.png"/>
</p>

### Description

Configures and reads the data stream for a Semantic Camera sensor and outputs a picture of the specified size on the Camera configuration.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Image:**  Image output with the format and dimensions  specified in
the sensor configuration.
 

- **Raw data:**  Array with pixel values for image. Size is equal to width x
height
 

- **pixel:**  Pixel value
 

- **Camera configuration:**  Settings to configure a semantic camera
 

- **Annotation data:**  Annotation data for all objects classified on the image
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
