# mono_parse_map_data.vi

<p class="img_container">
<img class="lg_img" src="../mono_parse_map_data.png"/>
</p>

### Description

Parse the map information depending on if this is in array format or geojson format.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **Map:**  Map string obtained from the server
 

- **Array?:**  True if the **Map** contains the map information in array
format. False indicates the  **Map** is in Geojson format
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Lanes:** Information lanes, including:   
    - x coordinates
    - y coordinates
    - z coordinates 
- **Map Geojson:**  Post processed Geojson map, remove extra quotes added on the
serialization.
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
nbsp;</p>
