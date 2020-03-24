## mono_send_simulator_config.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/utilities/mono__send__simulator__configc.png" 
width="400"  />
</p>

### Description 
Configures simulator given a path to a  JSON file. The file will configure server ip, port, etc. The running mode, map to use and traffic
conditions i.e. number of vehicles to spawn.

### Inputs
- **Simulator Configuration:** JSON string with the Simulator configuration.
- **monoDrive in(Cluster):** See description at monoDrive.ctl. 
- **Running mode(Enum):** One od the running modes Closed_loop, Replay, HIL  .
- **Select a map(Enum) :** One of the monoDrive maps or any user-created name map .
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **monoDrive out (Cluster):** See description at monoDrive.ctl. 
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
