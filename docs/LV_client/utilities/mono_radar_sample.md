# mono_radar_sample.vi

<p class="img_container">
<img class="lg_img" src="../mono_radar_sample.png"/>
</p>

### Description

Format the Radar sample into the final format for user display. Include the number of target detected.

For technical support contact us at **monodrive.support@ni.com** 

### Inputs

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
e radar sensor.
 

- **target_list:**  Objects detected by radar.  An array of clusters with the
following structure.   

| Type  | Name   | Units/Description   |
| --------- | ------------ |------------ |
| SGL  | aoa | degrees |
|SGL | range  | meters |
|SGL | rcs | m^2 |
|SGL | velocity | m/s |
|1D Array String | target_ids | Ground truth annotation |
--- 

- **Target:**  Target detect by radar
 

- **aoa:**  Angle of arrival
 

- **range:**  Distance to the target
 

- **rcs:**  Radar cross-section 
 

- **velocity:**  Velocity of the target detected
 

- **target_ids:**  Possible ground truth label
 

- **target_id:**  Ground truth label
 

- **gt_targets:**  Ground truth information for objects on the scene. An array
of clusters with the following structure.      

| Type  | Name   | Units/Description   |
| --------- | ------------ |------------ |
| SGL  | aoa | degrees |
|SGL | range  | meters |
|SGL | rcs | m^2 |
|SGL | velocity | m/s |
|1D Array String | target_ids | Ground truth annotation |   
--- 

- **Target:**  Target detect by ground truth radar
 

- **aoa:**  Angle of arrival
 

- **range:**  Distance to the target
 

- **rcs:**  Radar cross-section 
 

- **velocity:**  Velocity of the target detected
 

- **target_ids:**  Ground truth label
 

- **target_id:**  Ground truth label
 


- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
