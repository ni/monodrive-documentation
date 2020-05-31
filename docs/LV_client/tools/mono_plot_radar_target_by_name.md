# mono_plot_radar_target_by_name.vi

<p class="img_container">
<img class="lg_img" src="../mono_plot_radar_target_by_name.png"/>
</p>

### Description

Tool used to generate plot to compare AoA, Range and Velocity between the GT targets and the target list from the Radar sensor. This tool only tracks one target by the specified target on the **Name of Target** control 

### Inputs

- **Radar Sample:**  Refer to the **radarSample.ctl** description 

- **Name of target:**  Name of the target to track and generated plots. This target
name will be used to generate the AoA, Range and Velocity
plots comparing the Ground Truth information and the
information returned by the sensor
 

- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Average AoA:**  Average all the AoA with the tracked tag on the target_list
 

- **Average GT AoA:**  Average all the AoA with the tracked tag on the gt_targets
 

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
