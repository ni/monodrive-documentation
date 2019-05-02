## mono_get_lidar_setting.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/blob/master/WikiPhotos/LV_client/utilities/mono__get__lidar__settingsc.png" 
width="400"  />
</p>

### Description 
Read sensor configuration and obtain horizontal resolution and number of lasers.

### Inputs

- **Sensor Config (Reference) :** Reference to the Sensor Config path, used to set default directory .
- **Lidar_configured(Boolean):** True if Lidar was configured .
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **cycles(I32) :** Number of packets send thru UDP.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
