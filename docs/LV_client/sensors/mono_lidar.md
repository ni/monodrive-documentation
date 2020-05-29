# mono_lidar.vi

<p class="img_container">
<img class="lg_img" src="../mono_lidar.png"/>
</p>

### Description

Configure and procces the data stream for LiDAR sensor. This VI also forward the raw data to port 2368 thru TCP to be visualized by VeloView.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Raw data:**  Array of data with the unparsed data from the simulator
 

- **raw packet:**  Unparsed data from the simulator
 

- **Lidar Configuration:**  Configuration used to setup LiDAR sensor
 

- **Timestamp at zero azimuth:**  Timestamp at zero azimuth
 

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
