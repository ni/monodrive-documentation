## mono_matlab_plot_radar.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/tools/mono__matlab__plot__radarc.png"  /mono__matlab__plot__radarc.png" </p>

### Description 
Process radar data cube and run a Matlab script to plot Radar data.

### Inputs

- **radar data cube (String):** Raw data.
- **Radar_data (Cluster):** Cluster with the processed data for the Radar sensor.

| Type  | Name   |
| ------------ | ------------ |
|I32  | time_stamp |
|I32 | game_time  |
|1D Array SGL | ranges  |
|1D Array SGL  | aoa_list |
|1D Array SGL | velocities |

- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs

- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
