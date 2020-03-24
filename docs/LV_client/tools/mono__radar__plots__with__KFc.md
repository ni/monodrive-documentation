## mono_radar_plots_with_KF.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/tools/mono__radar__plots__with__KFc.png" 
width="400"  />
</p>

### Description 
Similar to **mono_radar_plots.vi** this tools is used to plot the ranges, aoa and velocities for a specific target to compare outputs for the Ground Truth and the monoDrive radar. 
This tool also add a KF for the output of the AoA to smooth the output.  

### Inputs
- **Radar GT:** The radar cluster sample for the Ground Truth.
- **Radar:** The monoDrive radar cluster sample.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
