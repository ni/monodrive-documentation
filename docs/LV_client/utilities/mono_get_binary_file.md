# mono_get_binary_file.vi

<p class="img_container">
<img class="lg_img" src="../mono_get_binary_file.png"/>
</p>

### Description

 

### Inputs

- **Sensor configuration:**  Path to the configuration file created by the mono_logger.vi
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Channels:**  If the sensor configurated was a camera, this VI will
extracted the number of channels from the the configuration
file
 

- **Type:**  Type of sensor obtained from the sensor configuration
 

- **Stream dimensions:**  If the type of sensor is a camera returned the stream
dimensions to parse image correctly.
 

- **Binary File ref:**  Reference to the binary file found based on the sensor
configuration 
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
