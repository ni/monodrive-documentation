## mono_GrayScaleToSimulink.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/mono__GrayScaleToSimulink.png" width="400"  />
</p>

### Description 
Used to send a Grayscale image to matlab.
### Inputs

- **Radar configuration (1D Array U8) :** An array with the pixel values of the image to be sent.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **send_radar_cube (Boolean) :** Indicates if the flag is present or not.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
