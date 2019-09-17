## mono_parse_lidar_configuration.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/mono__parse__lidar__configurationc.png" 
width="400"  />
</p>

### Description 
Calculates the number of packets an the number of points in a lidar stream based on the horizontal resolution.

### Inputs

- **Lidar configuration(String):** Lidar JSON configuration text.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **No.packets(I32):** Number of data packets in the stream depending on the horizonta resolution.
- **No.points(I32) :** Number of data points on the stream depending on the horizonta resolution.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
