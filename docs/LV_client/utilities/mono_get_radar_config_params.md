# mono_get_radar_config_params.vi
Parses the configuration and obtains the following parameters:  elements, the number of sweeps  and  number of samples per sweep.

<p class="img_container">
<img class="lg_img" src="../mono_get_radar_config_params.png"/>
</p>

### Description

 

### Inputs

- **Radar configuration:**  Radar configuration (JSON)
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **elements:**  The total number of receiver elements for the Radar.
 

- **num_sweeps:**  The total number of scanning sweeps to make per update.
 

- **num_samples_per_sweep:**  The total number of shooting bouncing rays to trace for a
single Radar sweep.
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
