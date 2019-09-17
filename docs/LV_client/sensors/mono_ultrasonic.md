## mono__ultrasonic.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__ultrasonicc.png" 
width="400"  />
</p>

### Description
Configures and sample the ultrasonic sensor according to the configuration setting.


### Inputs

- **monoDrive in (Cluster):** See description at **monoDrive.ctl**.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **Ultrasonic Range:** Distance in meters of the objects detected.
- **Ultrasonic configuration:** Outputs the configuration for the **ultrasinc** sensor sensor.
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
