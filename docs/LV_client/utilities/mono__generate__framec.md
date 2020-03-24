## mono_generate_frame.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/utilities/mono__generate__framec.png" 
width="400"  />
</p>

### Description 
Select a frame from a list of frames and generate a JSON string to send to the simulator.

### Inputs
- **Frames(Array of Cluster):** .
- **Send frame number(I32) :** .
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **frame(String):** A JSON string with the position of the EGO vehicle and other vehicles on the scene.
- **Done(Boolean):** True if this is the last frame.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
