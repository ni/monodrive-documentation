## mono_lidar.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/blob/lv_client_docs/WikiPhotos/LV_client/sensors/monoDrive_lvlib_mono__lidarc.png?raw=true" 
width="400"  />
</p>

### Description
Gets data from Lidar parse it  and forward it to port 2368 (VeloView)

### Inputs

- **Array_Lidar (1D Array of Clusters) :** Collection of Lidar sensors configured.
  - Cluster with 2 elements:
    * connection ID (TCP Network connection): Connection ID corresponding to the TCP connection for this **Lidar** sensor.
    * port (int): System port corresponding to this Lidar sensor.
- **Index (Int):** Index of the elemenet in the **Array_Lidar** you are interested.
- **UDP port (UDP Network Connection):** UDP connection to the port where VeloView is listening, typically 2368.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs

- **lidar raw data (String)** Output connection ID corresponding to the TCP connection for **Lidar** sensor, same as input connection ID.
- **lidar_data (Cluster):** Cluster with the processed data for the Lidar sensor.

| Type  | Name   |
| ------------ | ------------ |
|U32  | lenght |
|U32 | time_stamp  |
|SGL | game_time  |

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

