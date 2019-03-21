## mono_imu.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/monoDrive_lvlib_mono__imuc.png" 
width="400"  />
</p>

### Description
Reads the data stream for the IMU sensor and outputs the IMU sample.

### Inputs

- **Array_IMU (1D Array of Clusters) :** Collection of IMU sensor configured.
  - Cluster with 2 elements:
    * connection ID (TCP Network connection): Connection ID corresponding to the TCP connection for this **IMU** sensor.
    * port (int): System port corresponding to this IMU sensor.
- **Index:** Index of the elemenet in the **Array_IMU** you are interested.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs

- **Connection ID Out (TCP Network connection):** Output connection ID corresponding to the TCP connection for **IMU** sensor, same as input connection ID.
- **imu_sample:** Cluster with the processed data for the IMU sensor.

| Type  | Name   |
| ------------ | ------------ |
|U8  | packetSize |
|SGL | accel_x  |
|SGL | accel_x  |
|SGL | accel_z |
|SGL | angle_rate_x |
|SGL | angle_rate_y  |
|SGL | angle_rate_z  |
|U32 | timer |
|U16 | checksum|
|I32 | timeWeek  |

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
