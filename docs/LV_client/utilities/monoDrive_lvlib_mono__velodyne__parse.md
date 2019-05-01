## mono_lidar.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/monoDrive_lvlib_mono__sensor__parserc.png" 
width="400"  />
</p>

### Description
Parse UDP packet from Velodyne HDL.

### Inputs
- **Raw LIDAR Data:** Raw data from the lidar stream.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs
- **Parsed LIDAR Data(Array of Clusters):** Array of clusters with the processed data for the Lidar sensor.

| Type  | Name   |
| ------------ | ------------ |
|DBL  | Rotational Angle (degrees) |
|DBL | Vertical Angle (degrees)  |
|DBL | Distance (cm)  |
|U8 | Intensity (0-255)  |
|DBL | Vertical Offset (cm)  |

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

