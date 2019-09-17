## mono_get_config_path.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/mono__get__config__pathc.png" 
width="400"  />
</p>

### Description 
Check if a path is relative or absolute. Decide to pass the name of the file or the path.

### Inputs

- **Config_Path:** Path to a configuration file, by default point to $(User Home directory)/monoDriveConfig .
- **Base_Path:** $(User Home directory)/monoDriveConfig. The directory gets created when installing the package.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **Config_Path_out:** Path to the configuration file.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
