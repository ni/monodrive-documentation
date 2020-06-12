# mono_semantic_lidar.vi

<p class="img_container">
<img class="lg_img" src="../mono_semantic_lidar.png"/>
</p>

### Description

Configure and process the data stream for a semantic LiDAR sensor. 

This VI forwards the raw data to port 2368 by default through TCP and visualized by VeloView. Change the port to a unique number to visualize more than one LiDAR.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Raw data:**  Array of data with the unparsed data from the simulator
 

- **Lidar Configuration:**  Configuration used to setup a semantic LiDAR sensor
 

- **Timestamp at zero azimuth:**  Timestamp at zero azimuth
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
