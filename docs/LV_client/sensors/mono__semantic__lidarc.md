## mono_semantic_lidar.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__semantic__lidarc.png" 
width="400"  />
</p>

### Description
Configure and procces the data stream for a semantic LiDAR sensor. 

This VI also forward the raw data to port 2368  (by default) thru TCP to be visualized by VeloView. Change the port to new port if need to visualize more than one LiDAR.
 
### Inputs
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs
- **Raw data(Array of String)** Array containing all the raw data in the number of packets expected according to the **horizontal_resolution** specified on the **Lidar configuration**.
- **Lidar Configuration (String):** Configuration used for lidar.
- **time_zero (U32):** Timestamp of the first azimuth.
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.