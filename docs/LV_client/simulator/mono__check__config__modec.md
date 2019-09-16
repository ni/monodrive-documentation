## mono_check_config_mode.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/simulator/mono__check__config__modec.png" 
width="400"  />
</p>

### Description
Replaces the running mode from the configuration text with the running mode  chosen by the user.

### Inputs

- **configuration text (String) :** Text obtain by reading the Simulator configuration file .
- **Running mode (Enum)** 
    * 0 - Closed_loop 
    * 1 - Replay
    * 2 - Replay_step 
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **config_text_out (String) :** Text with the simulator mode setting replaced with the Running mode .
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
