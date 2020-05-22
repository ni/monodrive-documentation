## mono__semantic__camera.vi
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__semantic__camerac.png"/>
</p>

### Description
Configures and reads the stream data for a semantic camera sensor and outputs a picture of the size specified on the **Camera configuration**.

### Inputs
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **Camera configuration(String):** Camera configuration.
- **Image (Picture):** Procesed image, ready for display.
- **Raw data (2D U8 array):** Pixel values for the images .
- **Annotation data(String):** Classification data for elements in the image (Optional). 
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>
