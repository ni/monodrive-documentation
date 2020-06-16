# mono_ultrasonic.vi

<p class="img_container">
<img class="lg_img" src="../mono_ultrasonic.png"/>
</p>

### Description

Configures and samples the ultrasonic sensor according to the configuration settings. Provides range from the closest object to a single ultrasonic sensor.

For technical support contact us at <b>support@monodrive.io</b>
 

### Inputs

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Ultrasonic configuration:**  Settings used to configure a Ultrasonic sensor
 

- **Ultrasonic Range:**  Provides range to closest object from a single ultrasonic
sensor.
 

- **targets:**  Objects detected by the ultrasonic sensor
 

- **data:**  Include range information
 

- **range:**  Distance in meters of the objects detected. Returns -1 if
there is no object detected
 

- **Raw data:**  Unparsed data from simulator
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
