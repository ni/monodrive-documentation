# mono_weather_id.vi

<p class="img_container">
<img class="lg_img" src="../mono_weather_id.png"/>
</p>

### Description 
Obtains the weather id name selected by the user.

### Inputs

- **monoConfig in(Cluster):** Refer to monoConfig cluster definition.

- **Weather Profile Ref (Reference):** Reference of the array with the weather configuration .

- **External Weather Control (Bool):** True to change the weather profile using TestStand or other external tool.

- **Weather Index (Int):** Index selected by TestStand to configure weather.

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **Weather profiles(Cluster):** Refer to monoConfig cluster definition.

- **monoConfig out(Cluster):** Refer to monoConfig cluster.

- **id:** Name of the weather profile chosen.

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>