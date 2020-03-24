## mono_radar.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__radarc.png" width="400"  />
</p>

### Description
Configure and reads the stream data for the Radar sensor and outputs a outputs a cluster with three 1D arrays for **Ranges**, **aoas** and **velocities**.

### Inputs
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs
- **Radar configuration(String):** Settings specific for the Radar detection.
- **Raw data:** The unparsed data directly from the sensor.
- **Radar Data Cube (String):** Raw data for a cube of dimensions M (number of array antenna inputs) by L (number of range bins in fast time) by N (number of pulse in CPI in slow time).
- **Radar Sample (Cluster):** Cluster with the processed data for the Radar sensor.

| Type  | Name   |
| ------------ | ------------ |
|I32  | time_stamp |
|I32 | game_time  |
|1D Array SGL | ranges  |
|1D Array SGL  | aoa_list |
|1D Array SGL | velocities |

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

