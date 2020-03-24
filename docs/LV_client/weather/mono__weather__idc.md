## mono_weather_id.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/weather/mono__weather__idc.png" 
width="400"  />
</p>

### Description 
Obtains the weather id name selected by the user.

### Inputs

- **monoConfig in(Cluster):** Refer to monoConfig cluster definition.
- **Array in (Reference):** Reference of the array with the weather configuration .
- **TestStand (Boolean):** True if the VI is running from TestStand.
- **Weather Index (Int):** Index selected by TestStand to configure weather.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **Weather profiles(Cluster):** Refer to monoConfig cluster definition.
- **monoConfig out(Cluster):** Refer to monoConfig cluster.
- **id:** Name of the weather configuration.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
