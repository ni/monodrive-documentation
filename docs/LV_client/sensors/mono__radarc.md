## mono_radar.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__radarc.png" 
width="400"  />
</p>

### Description
Reads the stream data for the Radar sensor and outputs a 3D array with the formatted data.

### Inputs

- **sensor variants (Variant):** Contains the camera information for the **radar** sensor.
- **Port number (String):** Port number where the **radar** sensor is connected.d.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs
- **sensor variants out (Variant):** Contains the camera information for the **radar** sensor.
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

