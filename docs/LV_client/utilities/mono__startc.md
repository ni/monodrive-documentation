## mono_start.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/monoDrive_lvlib_mono__startc.png?raw=true" 
width="400"  />
</p>

### Description 
Stablish connection to the simulator, sends configuration and weather configuration.

### Inputs

- **Array in (Reference):** Reference to weather array .
- **Running mode (Enum):** Running mode to configure simulator. 0- Closed_loop, 1-Replay, 2-hil .
- **Trajectory configuration(String) :** Trajectory JSON text.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **weather profiles(Array of clusters) :** Contains the weather profiles configured.
- **monoDrive out (monoDrive.ctl) :** Cluster with the monoDrive data.
- **id (String):** Weather id selected .
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
