## mono_lidar.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__lidarc.png" width="400"  />
</p>

### Description
Gets data from Lidar parse it  and forward it to port 2368 (VeloView)

### Inputs
- **monoDrive in (Cluster):** See description at **monoDrive.ctl**.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs
- **Raw data(Array of String)** Array containing all the raw data in the number of packets expected according to the **horizontal_resolution** specified on the **Lidar configuration**.
- **Lidar Configuration (String):** Configuration used for lidar.
- **time_zero (U32):** Timestamp of the first azimuth.
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

