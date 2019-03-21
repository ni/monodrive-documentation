## mono_radar.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/blob/lv_client_docs/WikiPhotos/LV_client/sensors/monoDrive_lvlib_mono__radarc.png?raw=true" 
width="400"  />
</p>

### Description
Reads the stream data for the Radar sensor and outputs a 3D array with the formatted data.

### Inputs

- **Array_Radar (1D Array of Clusters) :** Collection of Radar sensor configured.
  - Cluster with 2 elements:
    * connection ID (TCP Network connection): Connection ID corresponding to the TCP connection for this **Radar** sensor.
    * port (int): System port corresponding to this Radar sensor.
- **Index:** Index of the elemenet in the **Array_Radar** you are interested.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs

- **Connection ID Out (TCP Network connection):** Output connection ID corresponding to the TCP connection for **Radar** sensor, same as input connection ID.
- **radar_output(Cluster):** Cluster with the processed data for the Radar sensor.

| Type  | Name   |
| ------------ | ------------ |
|I32  | time_stamp |
|I32 | game_time  |
|1D Array SGL | ranges  |
|1D Array SGL  | aoa_list |
|1D Array SGL | velocities |

- **radar data cube (String):** Raw data.

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

