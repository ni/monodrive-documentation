## mono_imu.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/monoDrive_lvlib_mono__imuc.png" 
width="400"  />
</p>

### Description
Reads the data stream for the IMU sensor and outputs the IMU sample.

### Inputs

- **sensor variants (Variant):** Contains the camera information for the **imu** sensor.
- **Port number (String):** Port number where the **imu** sensor is connected.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs

- **sensor variants out (Variant):** Contains the camera information for the **imu** sensor.
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
