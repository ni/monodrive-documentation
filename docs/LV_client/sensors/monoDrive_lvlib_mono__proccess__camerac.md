## mono_gps.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/blob/lv_client_docs/WikiPhotos/LV_client/sensors/monoDrive_lvlib_mono__proccess__camerac.png?raw=true" 
width="400"  />
</p>

### Inputs

- **Array_Camera (1D Array of Clusters) :** Collection of Camera sensors configured.
  - Cluster with 2 elements:
    * connection ID (TCP Network connection): Connection ID corresponding to the TCP connection for this **Camera** sensor.
    * port (int): System port corresponding to this Camera sensor.
- **Camera dimensions (1D Array of Clusters):** Array of clusters with the dimensions for each camera.
  - Cluster with two elements:
    * "x dimension" (I32) 
    * "y dimension" (I32).
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs

- **camera array (Cluster):** Cluster of pictures, corresponding to each camera.  
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

