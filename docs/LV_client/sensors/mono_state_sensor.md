# mono_state_sensor.vi

<p class="img_container">
<img class="lg_img" src="../mono_state_sensor.png"/>
</p>

### Description

Configure and read the State sensor information of the vehicle. The State sensor provides dynamic and spatial information of the elements in the scene during simulation.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **State sensor sample:**  State sensor sample. Includes state information for actors
with the "desired_tags" and cull out information of actors
with the "undesired_tags"
 

- **State sensor configuration:**  Settings used to configure a State sensor
 

- **Raw data:**  Unparsed data from simulator
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
