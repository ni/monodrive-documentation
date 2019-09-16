## mono_weather_updatec.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/weather/monoDrive_lvlib_mono__weather__updatec.png" 
width="400"  />
</p>

### Description 
Sends weather configuration to the simulator.

### Inputs

- **TCP Network Connection in (TCP Network connection) :** TCP connection to the server .
- **weather id (string):** Name of the weather configuration selected.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **weather profiles(1D array of Clusters) :** Cluster with all the profiles for the weather configuration.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
