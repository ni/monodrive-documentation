## mono_gps.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/blob/lv_client_docs/WikiPhotos/LV_client/sensors/monoDrive_lvlib_mono__lidarc.png?raw=true" 
width="400"  />
</p>

### Inputs

- **Array_Lidar (1D Array of Clusters) :** Collection of Lidar sensors configured.
  - Cluster with 2 elements:
    * connection ID (TCP Network connection): Connection ID corresponding to the TCP connection for this **Lidar** sensor.
    * port (int): System port corresponding to this Lidar sensor.
- **Index:** Index of the elemenet in the **Array_Lidar** you are interested.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs

- **lidar raw data** Output connection ID corresponding to the TCP connection for **Lidar** sensor, same as input connection ID.
- **lidar_data:** Cluster with the processed data for the Lidar sensor.

| Type  | Name   |
| ------------ | ------------ |
|U32  | lenght |
|U32 | time_stamp  |
|SGL | game_time  |

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

