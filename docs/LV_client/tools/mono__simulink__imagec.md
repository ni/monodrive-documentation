## mono_simulink_image.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/tools/mono__simulink__imagec.png" width="400"  />
</p>

### Description 
Send a RGB image to Simulink.
### Inputs

- **rgb_image(Array of Int):** Array of int with the RGB values of an image.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **forward_Amount:** Forward amount response from the Simulink module.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
