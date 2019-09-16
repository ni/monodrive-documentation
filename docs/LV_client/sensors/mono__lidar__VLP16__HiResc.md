## mono_lidar_VLP16_HiResc.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__lidar__VLP16__HiResc.png" 
width="400"  />
</p>

### Description
Gets data from Lidar parse it  and forward it to port 2368 (VeloView)

### Inputs
- **Veloview UDP port (UDP Network Connection):** UDP connection to the port where VeloView is listening, typically 2368.
- **sensor variants (Variant):** Contains the camera information for the **lidar** sensor.
- **Port number (String):** Port number where the **lidar** sensor is connected.
- **Veloview Address (Int):** UDP connection, typically 2367.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs
- **sensor variants out (Variant):** Contains the camera information for the **camera** sensor.
- **lidar_data (Cluster):** Cluster with the processed data for the Lidar sensor.

| Type  | Name   |
| ------------ | ------------ |
|U32  | lenght |
|U32 | time_stamp  |
|SGL | game_time  |

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


