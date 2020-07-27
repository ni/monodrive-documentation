# mono_parse_depth_data.vi

<p class="img_container">
<img class="lg_img" src="../mono_parse_depth_data.png"/>
</p>

### Description

Parse the raw data from the Depth Camera sensor and output the  normalized data.

For technical support contact us at **support@monodrive.io** 

### Inputs

- **Distance (meters):**  Adjust the maximum distance an object can be seen
 

- **Data:**  Raw binary data from the sensor
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Raw data:**  Data parsed into an array of double.
 
- **Normalized data:** Data normalized between 0 and 255, corresponding to the depth information.


- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
