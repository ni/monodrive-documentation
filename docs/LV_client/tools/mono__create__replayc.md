## mono_create_replay.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/tools/mono__create__replayc.png" 
width="400"  />
</p>

### Description 

### Inputs
- **Path to save(Path):** The path to save the replay.json generated.
- **Frames(1D Array String ):** a Array with all the frames from the state sensor.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **Path to save(Path):** The path to the generated replay.json. 
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
