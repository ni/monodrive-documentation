## mono_check_config_mode.vi
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/simulator/mono__check__config__modec.png"/>
</p>

### Description
Build the simulator configuration with the running mode chosen by the user.

### Inputs

- **configuration text (String) :** Base simulator configuration file .
- **Running mode (Enum)** 
    * 0 - Closed_loop 
    * 1 - Replay
    * 2 - Replay_step 
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **config_text_out (String) :** Text with the simulator mode setting replaced with the Running mode .
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>