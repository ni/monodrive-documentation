# mono_get_map.vi

<p class="img_container">
<img class="lg_img" src="../mono_get_map.png"/>
</p>

### Description

Send the GetMap command to obtain the map data in GeoJSON or array format.

For technical support contact us at <b>monodrive.support@ni.com</b> 

### Inputs

- **message:**  Message for the **GetMap** server command.    
**"format"** can take values: "point_array" or "geojson"   
**"coordinates"** can take values : "world" or "gis"
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
 

- **monoDrive in (Cluster):** See description at [**monoDrive.ctl**](../structures/monoDrive.md). 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
