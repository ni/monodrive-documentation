## mono_send_simulator_config.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/mono__send__simulator__configc.png" 
width="400"  />
</p>

### Description 
Configures simulator given a path to a  JSON file. The file will configure server ip, port, etc.

### Inputs

- **monoDrive.ctl (Cluster):** See description at **monoDrive.ctl**.
- **Running mode (Enum)** 
    * 0 - Closed_loop 
    * 1 - Replay
    * 2 - Replay_step 
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

### Outputs

- **monoDrive.ctl (Cluster):** See description at **monoDrive.ctl**.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
