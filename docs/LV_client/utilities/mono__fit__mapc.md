## mono_fit_map.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/utilities/mono__fit__mapc.png" 
width="400"  />
</p>

### Description 
Tool used to draw a point on the scaled map.

### Inputs
- **x:** The x location on the world coordinates.
- **y:** The y location on the world coordinates. .
- **DrawArea:** The size of the area to draw the map.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **point(x,y):** The coodinates x,y on the scale map.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
