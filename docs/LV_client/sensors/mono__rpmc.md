## mono_rpm.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/monoDrive_lvlib_mono__rpmc.png" 
width="400"  />
</p>

### Description
Get the RPM information of the vehicle.

### Inputs

- **sensor variants (Variant):** Contains the camera information for the **RPM** sensor.
- **port number (String):** Port number where the **RPM** sensor is connected.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs
- **sensor variants out (Variant):** Contains the information for the **RPM** sensor.
- **wheel_rpm_sample:** Cluster with the processed data for the RPM sensor.

| Type  | Name   |
| ------------ | ------------ |
|U32  | wheel_number |
|U8 | wheel_speed_lf  |
|U8 | wheel_speed_rf  |
|U8 | wheel_speed_lr |
|U8 | wheel_speed_rr |

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

