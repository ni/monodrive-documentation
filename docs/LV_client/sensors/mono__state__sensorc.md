## mono_state_sensor.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__state__sensorc.png" 
width="400"  />
</p>

### Description
Get the state sensor information of the vehicle.

### Inputs

- **sensor variants (Variant):** Contains the camera information for the **State Sensor** sensor.
- **port number (String):** Port number where the **State Sensor** sensor is connected.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs
- **sensor variants out (Variant):** Contains the information for the **State Sensor** sensor.
- **State Sensor output:** JSON format text with information of all the vehicles in the frame.
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

