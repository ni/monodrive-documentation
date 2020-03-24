## mono_get_map.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/utilities/mono__get__mapc.png" 
width="400"  />
</p>

### Description 
Send the command Get_Map to the server to obtain the map information on geojson or array format.

### Inputs
- **monoDrive in (Cluster):** See description at monoDrive.ctl. 
- **message(String):** The JSON string specifying the format of the data returned. ie.
``` 
{		"gis_anchor": {
			"x": 40.410262,
			"y": -79.948172,
			"z": 1
		},
		"orientation": -16.5,
		"point_delta": 100.0,
		"coordinates": "world",
   "format": "point_array"
}

```
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **monoDrive out (Cluster):** See description at monoDrive.ctl
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
