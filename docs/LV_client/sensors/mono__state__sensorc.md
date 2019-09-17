## mono_state_sensor.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__state__sensorc.png" 
width="400"  />
</p>

### Description
Get the state sensor information of the vehicle.

### Inputs

- **monoDrive.ctl (Cluster):** See description at **monoDrive.ctl**.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs
- **State Sensor sample:** JSON format text with information of all the vehicles in the frame.
- **IMU Configuration(String):** Configuration used for State Sensor.
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

