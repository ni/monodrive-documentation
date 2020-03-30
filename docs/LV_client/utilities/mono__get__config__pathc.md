## mono_get_config_path.vi
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/mono__get__config__pathc.png"   />
</p>

### Description 
Check if a path is relative or absolute. Decide to pass the name of the file or the path.

### Inputs

- **Config_Path:** Path to a configuration file, by default point to $(User Home directory)/monoDriveConfig .
- **Base_Path:** Base path to build an absolute path. 
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **Config_Path_out:** Path to the configuration file.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>