## mono_convert_polar_to_rectangular.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/monoDrive_lvlib_mono__convert__polar__to__rectangularc.png" 
width="400"  />
</p>

### Description 
Converts polar coordinates from the Velodyne to rectangular.

### Inputs

- **Laser Packet(Cluster) :** .

| Type  | Name   |
| ------------ | ------------ |
|DBL  | Rotational Angle (degrees) |
|DBL | Vertical Angle (degrees)  |
|DBL | Distance (cm)  |
|U8 | Intensity (0-255)  |
|DBL | Vertical Offset (cm) |

- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **Cartesians Coordinates (Cluster) :** A cluster with cartesian coordinate x,y,z .

| Type  | Name   |
| ------------ | ------------ |
|SGL  | X |
|SGL | Y  |
|SGL | Z  |
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
