## .vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/monoDrive_lvlib_mono__intensity__to__colorc.png" 
width="400"  />
</p>

### Description 
Converts from a color number to its components in RGB.

### Inputs

- **Color Table (Array U32):** An array with a number corresponding to a color.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **RGBA array:** An array with Cluster "Color".
Cluster "Color"
| Type  | Name   |
| ------------ | ------------ |
|SGL  | Red |
|SGL| Green  |
|SGL| Blue  |
|SGL | Alpha  |

- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
