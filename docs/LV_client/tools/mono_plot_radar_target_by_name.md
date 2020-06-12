# mono_plot_radar_target_by_name.vi

<p class="img_container">
<img class="lg_img" src="../mono_plot_radar_target_by_name.png"/>
</p>

### Description

Tool generates a plot to compare AoA, Range and Velocity between the GT targets and the target list from the Radar sensor. This tool only tracks one target by the specified target on the Name of Target control. 

### Inputs

- **Radar Sample:**  Refer to the **radarSample.ctl** description 

- **Number of targets:**  Number of targets detected by Radar

- **target_list:**  **target_list**: Objects detected by radar.  Array of
clusters with the following structure.

| Type  | Name   | Units/Description   |
| --------- | ------------ |------------ |
| SGL  | aoa | degrees |
|SGL | range  | meters |
|SGL | rcs | m^2 |
|SGL | velocity | m/s |
|1D Array String | target_ids | Ground truth annotation |
--- 


- **aoa:**  Angle of arrival
 

- **range:**  Distance to the target
 

- **rcs:**  Radar cross-section 
 

- **velocity:**  Velocity of the target detected
 

- **target_ids:**  One or more possible labels for the object detected
 

- **gt_targets:**  **gt_targets**:  Array of clusters with Ground truth
information, each cluster has the following format.   

| Type  | Name   | Units/Description   |
| --------- | ------------ |------------ |
| SGL  | aoa | degrees |
|SGL | range  | meters |
|SGL | rcs | m^2 |
|SGL | velocity | m/s |
|1D Array String | target_ids | Ground truth annotation |
--- 

 
- **Name of target:**  Target name of tracking and generating plots. This target
name will be used to generate the AoA, Range and Velocity
plots that compare the Ground Truth information and the
information returned by the sensor
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Average AoA:**  Average all the AoA with the tracked tag on the target_list
 

- **Average GT AoA:**  Average all the AoA with the tracked tag on the gt_targets
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
de if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
