# mono_get_binary_file.vi

<p class="img_container">
<img class="lg_img" src="../mono_get_binary_file.png"/>
</p>

### Description

Obtain the reference to the binary file to read based on the name of the sensor type and the custom name assigned by the user. 

For technical support contact us at <b>monodrive.support@ni.com</b>  

### Inputs

- **Sensor configuration:**  Path to the configuration file created by the mono_logger.vi.
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Channels:**  If the sensor configured is a camera, this VI will
extract the number of channels from the the configuration
file.
 

- **Type:**  Type of sensor obtained from the sensor configuration.
 

- **Stream dimensions:**  If the sensor type is camera, the stream dimensions will be returned in order to parse the image correctly.
 

- **Binary File ref:**  Reference to the binary file found based on the sensor
configuration.
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
