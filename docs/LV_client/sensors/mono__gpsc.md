## mono_gps.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__gpsc.png" 
width="400"  />
</p>

### Description
Reads the data stream data for the GPS sensor and outpust a cluster with the  formatted data.

### Inputs

- **monoDrive.ctl (Cluster):** See description at **monoDrive.ctl**.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs
- **GPS Sample:** Cluster with the processed data for the GPS sensor.

| Type  | Name   |
| ------------ | ------------ |
|U8  | preamble |
|U16 | MSG_POS_LLH  |
|U16 | sensor_id  |
|U8  | payload_length |
|DBL | latitude |
|DBL | longitude  |
|DBL | elevation  |
|SGL | WorldLocation_x |
|SGL | WorldLocation_y|
|SGL | forward_x  |
|SGL | forward_y   |
|SGL | forward_z   |
|SGL | ego_yaw  |
|SGL | ego_speed |
|U16 | horizontal_acceleration  |
|U16 | vertical_acceleration  |
|U8  | satellites  |
|U8  | mode   |
|U16 | CRC  |

- **Raw data (String):** Raw data from the simulator.
- **GPS Configuration(String):** Configuration used for GPS.
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

