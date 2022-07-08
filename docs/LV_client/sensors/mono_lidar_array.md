# mono_lidar_array.vi

<p class="img_container">
<img class="lg_img" src="../mono_lidar_array.png"/>
</p>

### Description

Configure and process the data stream for an array LiDAR sensor. 

For technical support contact us at <b>monodrive.support@ni.com</b> 

### Inputs

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **LiDAR Configuration:**  Configuration used to setup LiDAR sensor
 

- **LiDAR Data:**  Array of clusters that contain information for each laser. 

| Type  | Name   |
|  |  |
|SGL  | time |
|SGL  | x|
|SGL  | y|
|SGL  | z|
|SGL  | intensity|
|SGL  | laser_id|
 

- **Raw data:**  Binary raw data for the array lidar. 
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
