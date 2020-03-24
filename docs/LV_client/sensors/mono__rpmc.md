## mono_rpm.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__rpmc.png" width="400"  />
</p>

### Description
Configure and read the RPM information of the vehicle.

### Inputs
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs
- **RPM Sample:** Contains the information for the **RPM** sensor.

| Type  | Name   |
| ------------ | ------------ |
|U32  | wheel_number |
|U8 | wheel speed left front  |
|U8 | wheel speed rear front  |
|U8 | wheel speed left rear |
|U8 | wheel speed rear rigth |

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

