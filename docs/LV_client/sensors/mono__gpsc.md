## mono_gps.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__gpsc.png" 
width="400"  />
</p>

### Description
Reads the data stream data for the GPS sensor and outpust a cluster with the  formatted data.

### Inputs

- **sensor variants (Variant):** Contains the camera information for the **camera** sensor.
- **Port number (String):** Port number where the **camera** sensor is connected.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs
- **sensor variants out (Variant):** Contains the camera information for the **camera** sensor.
- **gps_sample:** Cluster with the processed data for the GPS sensor.

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

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

