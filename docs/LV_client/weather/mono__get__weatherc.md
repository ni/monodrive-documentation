## mono_get_weather.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/weather/mono__get__weatherc.png" 
width="400"  />
</p>

### Description 
Obtains the weather configuration  from a file and populates a cluster with the configuration.

### Inputs

- **Weather Config Path:** Path to the weather configuration file.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **weather profiles :** Cluster with all the profiles for the weather configuration.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
