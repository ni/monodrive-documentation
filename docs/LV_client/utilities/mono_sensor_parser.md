# mono_sensor_parser.vi

<p class="img_container">
<img class="lg_img" src="../mono_sensor_parser.png"/>
</p>

### Description

Gets Sensor type and ports from the configuration text.

For technical support contact us at <b>monodrive.support@ni.com</b> 

### Inputs

- **Sensor configuration:**  Sensor configuration in JSON format
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Port out:**  Number obtained from the configuration. This is the port
number to open a TCP connection 
 

- **Stream dimensions:**  Width and height for the output image from any camera
sensor, values obtained from the configuration
 

- **include_annotation:**  True if the *include_annotation* tag was found and set to
true on the  sensor configuration (only applies to camera
sensors)
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
