## mono__camera__single.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__camerac.png" 
width="400"  />
</p>

### Description
Reads the stream data for a camera sensor  and returns a picture with the format specified in the stream dimensions.

### Inputs

- **monoDrive in (Cluster):** See description at **monoDrive.ctl**.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **Image (Picture):** Procesed image, ready for display.
- **Raw data :** Pixel values for the images .
- **Camera configuration :** Camera configuration used for the camera.
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
