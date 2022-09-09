# mono_parse_map_data.vi

<p class="img_container">
<img class="lg_img" src="../mono_parse_map_data.png"/>
</p>

### Description

Parse the map information depending on if this is in array format or GeoJSON format.

For technical support contact us at <b>monodrive.support@ni.com</b> 

### Inputs

- **Map:**  Map string obtained from the server
 

- **Array?:**  True if the **Map** contains the map information in array
format. False indicates the  **Map** is in GeoJSON format
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Lanes:** Information lanes, including:   
    - x coordinates
    - y coordinates
    - z coordinates 

- **Map GeoJSON:**  Post processed GeoJSON map, remove extra quotes added on the
serialization.
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
nbsp;</p>
