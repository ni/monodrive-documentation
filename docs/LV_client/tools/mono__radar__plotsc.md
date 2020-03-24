## mono_radar_plots.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/tools/mono__radar__plotsc.png" 
width="400"  />
</p>

### Description 
This tools is used to plot the ranges, aoa and velocities for a specific target to compare outputs for the Ground Truth 
and the monoDrive radar. 

### Inputs
- **Radar GT:** The radar cluster sample for the Ground Truth.
- **Radar:** The monoDrive radar cluster sample.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
