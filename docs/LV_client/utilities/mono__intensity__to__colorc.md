## mono_intensity_to_color.vi
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/mono__intensity__to__colorc.png"   />
</p>

### Description 
Converts from a color number to its components in RGB.

### Inputs

- **Color Table (Array U32):** An array with a number corresponding to a color.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **RGBA array:** An array with Cluster "Color".

| Type  | Name   |
| ------------ | ------------ |
|SGL  | Red |
|SGL| Green  |
|SGL| Blue  |
|SGL | Alpha  |

- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>