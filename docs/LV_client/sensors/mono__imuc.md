## mono_imu.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__imuc.png" width="400"  />
</p>

### Description
Reads and process the data stream for the IMU senso. Outputs the IMU data in a cluster.

### Inputs

- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs

- **IMU Sample:** Cluster with the processed data for the IMU sensor.

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

- **Raw data (String):** Raw data from the simulator.
- **IMU Configuration(String):** Configuration used for IMU.
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
