## mono__camera__single.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/monoDrive_lvlib_mono__camera__singlec.png" 
width="400"  />
</p>

### Description
Reads the stream data for a camera sensor  and returns a picture with the format specified in the stream dimensions.

### Inputs

- **connection ID (TCP Network connection):** connection ID corresponding to the TCP connection for **camera** sensor.
- **stream dimensions (Cluster):** Specify the dimension of the image obtain with a specific camera.
  - Cluster with two elements **"x dimension" (I32)** and **"y dimension" (I32)**.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **Connection ID Out (TCP Network connection):** Output connection ID corresponding to the TCP connection for **camera** sensor, same as input connection ID.
- **camera_out (picture):** Procesed image, ready for display .
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
