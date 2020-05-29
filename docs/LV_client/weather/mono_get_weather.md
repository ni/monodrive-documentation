# mono_get_weather.vi

<p class="img_container">
<img class="lg_img" src="../mono_get_weather.png"/>
</p>

### Description 
Obtains the weather configuration  from a file and populates a cluster with the configuration.

### Inputs

- **monoDrive in (Cluster):** See description at **monoDrive.ctl**.

- **Weather Profile Ref (Reference):** Reference to the Weather Profile array Indicator.

- **External Weather Control (Bool):** True to change the weather profile using TestStand or other external tool.

- **Weather Index:** Weather index to be changed by the automation tool like TestStand.

- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **monoDrive out (Cluster):** See description at **monoDrive.ctl**.

- **id:** Name of the weather profile chosen.

- **weather profiles :** Cluster with all the weather profiles.

- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>