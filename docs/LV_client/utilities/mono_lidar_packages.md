# mono_lidar_packages.vi

<p class="img_container">
<img class="lg_img" src="../mono_lidar_packages.png"/>
</p>

### Description

Calculate how many packages to send thu UDP based on the lidar configuration.
For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **Settings:**  Cluster with the settings for LiDAR:
    - Horizontal resolution (0.1 -0.8)
    - Number of laser  (16 laser or 32 laser only)
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Packets:**  Number of packets per point cloud
 

- **Number points:**  Number of points per point cloud
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
