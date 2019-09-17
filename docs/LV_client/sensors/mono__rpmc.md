## mono_rpm.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__rpmc.png" 
width="400"  />
</p>

### Description
Get the RPM information of the vehicle.

### Inputs

- **monoDrive.ctl (Cluster):** See description at **monoDrive.ctl**.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs
- **RPM Sample:** Contains the information for the **RPM** sensor.

| Type  | Name   |
| ------------ | ------------ |
|U32  | wheel_number |
|U8 | wheel_speed_lf  |
|U8 | wheel_speed_rf  |
|U8 | wheel_speed_lr |
|U8 | wheel_speed_rr |

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

