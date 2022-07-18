# mono_lidar.vi

<p class="img_container">
<img class="lg_img" src="../mono_lidar.png"/>
</p>

### Description

Configure and process the data stream for LiDAR sensor. This VI forwards the raw data to port 2368 through TCP to be visualized by VeloView.

For technical support contact us at <b>monodrive.support@ni.com</b> 

### Inputs

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Raw data:**  Array of data with the unparsed data from the simulator
 

- **raw packet:**  Unparsed data from the simulator
 

- **LiDAR Configuration:**  Configuration used to setup LiDAR sensor
 

- **Timestamp at zero azimuth:**  Timestamp at zero azimuth
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
