## mono_get_weather.vi
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/weather/mono__get__weatherc.png" />
</p>

### Description 
Obtains the weather configuration  from a file and populates a cluster with the configuration.

### Inputs

- **monoDrive in (Cluster):** See description at **monoDrive.ctl**.
- **Array in (Reference):** Reference to a array Indicator.
- **TestStand (Bool):** Indicate if the client is being run by testStand or other external tool.
- **Weather Index:** Weather index to be changed by the automation tool like TestStand.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **monoDrive out (Cluster):** See description at **monoDrive.ctl**.
- **id:** Weather id chosen.
- **weather profiles :** Cluster with all the profiles for the weather configuration.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>
