# mono_matlab_radar_plot.vi

<p class="img_container">
<img class="lg_img" src="../mono_matlab_radar_plot.png"/>
</p>

### Description

Process radar data cube and run a Matlab script to plot Radar cube data.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **Radar Data Cube:**  Only send if, **send_cube** is set to true on the **Radar
configuration**. Raw data for a cube of dimensions M (number
of array antenna inputs) by L (number of range bins in fast
time) by N (number of pulse in CPI in slow time).

**NOTE:** If the Radar data cube is empty (**send_cube** is set to false) the plot will be empty or won't show. 

- **Radar Sample:**  Radar sample contains information for the targets detected
by radar and ground truth information as well.
 

- **Radar configuration:**  Radar configuration (JSON)
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
