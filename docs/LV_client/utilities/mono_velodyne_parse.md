# mono_velodyne_parse.vi

<p class="img_container">
<img class="lg_img" src="../mono_velodyne_parse.png"/>
</p>

### Description

Parse UDP packet from Velodyne HDL.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **Raw LIDAR Data:**  Raw UDP packet from the Velodyne HDL.  This packet is always
1206 bytes and contains 384 laser returns.
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Pased LIDAR Data:**   

- **Laser Data Struct1:**   

- **Rotational Angle (degrees):**   

- **Vertical Angle (degrees):**   

- **Distance (cm):**   

- **Intensity (0-255):**   

- **Vertical Offset (cm):**   

- **timestamps:**   

- **timestamp (us):**   

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
of errors from other VIs. 

<p>&nbsp;</p>
