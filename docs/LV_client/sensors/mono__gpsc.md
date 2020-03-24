## mono_gps.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__gpsc.png" width="400"  />
</p>

### Description
Configure and reads the data stream data for the GPS sensor.

### Inputs

- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs
- **Raw data (String):** Raw data from the simulator. Output format as follows:

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
- **GPS Configuration(String):** Configuration used for GPS.
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

