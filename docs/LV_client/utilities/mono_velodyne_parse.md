# mono_velodyne_parse.vi

<p class="img_container">
<img class="lg_img" src="../mono_velodyne_parse.png"/>
</p>

### Description

Parse UDP packet from Velodyne HDL.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **Raw LiDAR data:**  Raw UDP packet from the Velodyne HDL.  This packet is always
1206 bytes.
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Pased LIDAR Data:**  List of points obtained from parsing the Raw LiDAR data
 

- **Timestamps:**  List of timestamps obtained from the raw LiDAR data
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
