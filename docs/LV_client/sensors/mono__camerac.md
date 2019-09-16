## mono__camera__single.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__camerac.png" 
width="400"  />
</p>

### Description
Reads the stream data for a camera sensor  and returns a picture with the format specified in the stream dimensions.

### Inputs

- **sensor variants (Variant):** Contains the camera information for the **camera** sensor.
- **Port number (String):** Port number where the **camera** sensor is connected.
- **stream dimensions (Cluster):** Specify the dimension of the image obtain with a specific camera.
  - Cluster with two elements **"x dimension" (I32)** and **"y dimension" (I32)**.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **sensor variants out (Variant):** Contains the camera information for the **camera** sensor.
- **TCP Network connection Out (TCP Network connection):** Output connection ID corresponding to the TCP connection for **camera** sensor, same as input connection ID.
- **camera_out (picture):** Procesed image, ready for display .
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
