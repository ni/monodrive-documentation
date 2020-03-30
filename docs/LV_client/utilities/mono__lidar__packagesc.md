## mono_lidar_packages.vi
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/mono__lidar__packagesc.png"   />
</p>

### Description 
Calculate how many packages to send thu UDP based on the lidar configuration.

### Inputs

- **settings(Cluster):** Settings for Lidar.

| Type  | Name   |
| ------------ | ------------ |
|DBL  | horizontal_resolution |
|I32 | n_lasers  |
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **cycles(I32) :** Number of packets send thru UDP.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>