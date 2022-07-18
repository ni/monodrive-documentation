# mono_convert_polar_to_rectangular.vi

<p class="img_container">
<img class="lg_img" src="../mono_convert_polar_to_rectangular.png"/>
</p>

### Description

Converts polar coordinates from the Velodyne to rectagular.

For technical support contact us at <b>monodrive.support@ni.com</b> 

### Inputs

- **Laser Packet:**  Polar cordinates for the points on a LiDAR packet    
    - Azimuth (degrees)
    - Vertical Angle (degrees)
    - Distance (centimeters)
    -  Intensity (0-255)
    - Vertical offset (centimeters)
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Cartesian Coordinates:**  Cartesian coordinates x,y,z
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
