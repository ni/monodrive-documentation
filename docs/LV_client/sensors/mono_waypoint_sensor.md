# mono_waypoint_sensor.vi

<p class="img_container">
<img class="lg_img" src="../mono_waypoint_sensor.png"/>
</p>

### Description

Configures and reads the Waypoint sensor information. The waypoint sensor returns a JSON string with information on the waypoints behind  and in front of all the vehicles in the scenario, given the specified distance and frequency.  Information about the left and right lanes is also included. 

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Waypoint sensor sample:**  The waypoint sensor sample includes information about the waypoints behind and in front of each actor on the scenario file according to the distance and frequency (how apart the waypoints are) specified on the Waypoint sensor configuration.
 

- **Waypoint sensor configuration:**  Settings used to configure a Waypoint sensor
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
