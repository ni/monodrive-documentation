## mono_gps.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/blob/lv_client_docs/WikiPhotos/LV_client/sensors/monoDrive_lvlib_mono__gpsc.png?raw=true" 
width="400"  />
</p>

### Inputs

- **Array_GPS (1D Array of Clusters) :** Collection of GPS sensor configured.
  - Cluster with 2 elements:
    * connection ID (TCP Network connection): Connection ID corresponding to the TCP connection for this **GPS** sensor.
    * port (int): System port corresponding to this GPS sensor.
- **Index:** Index of the elemenet in the **Array_GP** you are interested.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs

- **Connection ID Out (TCP Network connection):** Output connection ID corresponding to the TCP connection for **GPS** sensor, same as input connection ID.
- **gps sample:** Cluster with the processed data for the GPS sensor.

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

