# mono_lidar_plot.vi

<p class="img_container">
<img class="lg_img" src="../mono_lidar_plot.png"/>
</p>

### Description

Plot Lidar data into a 3D Plot Datatype.lvclass.

For technical support contact us at <b>monodrive.support@ni.com</b> 

### Inputs

- **LiDAR Data:**  Data for each point on the point cloud. Including:    
    - Azimuth (degrees)
    - Vertical Angle (degrees)
    - Distance (cm)
    - Intensity (0-255)
    - Vertical Offset (cm)
 

- **Number points:**  Number of points per point cloud

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **LiDAR points:**  List of points per point cloud 

- **Intensity:** RGBA values for each point based on intensity

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
