## mono_depth_camera.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__depth__camerac.png" width="400"  />
</p>

### Description 
Configures and reads the stream data for a  depth camera sensor and outputs a picture of the size specified on the **Camera configuration**.

### Inputs
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **Image (Picture):** Procesed image, ready for display.
- **Raw data (2D SGL array):** Pixel values for the images .
- **Annotation data (String):** Classification data for elements in the image.
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
