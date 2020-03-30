## mono__radar__ground__truth.vi
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__radar__ground__truthc.png"/>
</p>

### Description
Configures and reads the stream data for the Ground Truth Radar sensor and outputs a cluster with 3 1D arrays for **Ranges**, **aoas** and **velocities**.

### Inputs
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs
- **Radar Sample (Cluster):** Cluster with the processed data for the Radar sensor.

| Type  | Name   | Units   |
| ------------ | ------------ |------------ |
|I32  | time_stamp | seconds |
|I32 | game_time  | seconds |
|1D Array SGL | ranges  | meters |
|1D Array SGL  | aoa_list | degrees |
|1D Array SGL | velocities | m/s |

- **Radar Data Cube (String):** Raw data.
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>
