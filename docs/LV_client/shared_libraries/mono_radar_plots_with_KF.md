# mono_radar_plots_with_KF.vi

<p class="img_container">
<img class="lg_img" src="../mono_radar_plots_with_KF.png"/>
</p>

### Description

Tool to plot the range, aoa and velocity to compare the outputs of the mono_radar.vi and the mono_ground_radar.vi for a specific Tracking element on the list of target of the ground truth radar sensor. This tool also adds a Kalman Filter to the output of the  mono_radar.vi to smooth the aoa value. 

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **Radar Sample:**  Refer to the **radarSample.ctl** description 
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
