## mono_parse_map_data.vi
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/utilities/mono__parse__map__datac.png" 
  />
</p>

### Description 
Parse the map information depending on if this is in array format or geojson format.

### Inputs

- **data_in:** Raw map information.
- **Array?:** True if the information is in array format.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **lanes:** Cluster with the map information.
- **map_geojson:** Map string in geojson format.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>