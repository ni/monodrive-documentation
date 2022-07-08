# mono_parse_lidar_raw_data.vi

<p class="img_container">
<img class="lg_img" src="../mono_obtain_timestamp.png"/>
</p>

### Description

Obtains timestamp (position 1200) in microseconds from the raw LiDAR stream (1206 bytes).
For technical support contact us at <b>monodrive.support@ni.com</b> 

### Inputs

- **LiDAR raw data:**  Raw data form LiDAR sensor
 

- **Bit offset (1200):**  Bit offset where the timestamp information is located on the
LiDAR stream
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Timestamp (us):**  Timestamp in microseconds obtained from the LiDAR raw sample

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
