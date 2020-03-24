## mono_radar_data_cube_plotter.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/tools/mono__radar__data__cube__plotterc.png" 
width="400"  />
</p>

### Description 
Use to plot the FFT of the radar cube.

### Inputs

- **Radar cube(String):** Raw data cube from the simulator.
- **FFT size(I32):** Number of bins used for dividing the window into equal strips, or bins.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **#samples per frame(complex):** Samples per frame.
- **Fast Channel FFT:** Fast Channel FFT.
- **Sweep Response:** Sweep Response.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
