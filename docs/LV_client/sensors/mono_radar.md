# mono_radar.vi

<p class="img_container">
<img class="lg_img" src="../mono_radar.png"/>
</p>

### Description

Configure and reads the data stream for the Radar sensor.

For technical support contact us at <b>monodrive.support@ni.com</b>s 

### Inputs

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Radar Data Cube:**  Only send if, **send_cube** is set to true on the **Radar
configuration**. Raw data for a cube of dimensions M (number
of array antenna inputs) by L (number of range bins in fast
time) by N (number of pulse in CPI in slow time).
 

- **Radar configuration:**  Settings to configure a Radar sensor
 

- **Radar Sample:**  Radar sample contains information for the targets detected
by radar and ground truth information as well.
 

- **Number of targets:**  Number of targets detected by the radar sensor.
 

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
