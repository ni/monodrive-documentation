## mono_state_sensor.vi
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__state__sensorc.png"/>
</p>

### Description
Configure and read the state sensor information of the vehicle. The state sensor provides dynamic and spatial information of the elements in the scene during simulation.

### Inputs
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs
- **State Sensor sample:** JSON format text with information of all the vehicles in the frame.
- **State Sensor Configuration(String):** Configuration used for State Sensor.
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>
