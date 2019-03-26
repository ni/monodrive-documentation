## mono_config_directory_paths.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/monoDrive_lvlib_mono_config_directory_paths.png" 
width="400"  />
</p>

### Description 
Set the indicators to default state given a reference to the front panel indicators.

### Inputs
- **Sensor Config (Reference) :** Reference to the Sensor Config path, used to set default directory .
- **Trajectory Config (Reference):** Reference to the Sensor Trajectory path, used to set default directory .
- **Simulator Config (Reference):** Reference to the Simulator path, used to set default directory .
- **Simulator Configuration Path (Path):** Simulator configuration path.
- **Sensor Configuration Path (Path):**  Sensor configuration path.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **Configuration Paths (Cluster of Paths):**  Cluster with all the paths to the configuration files.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
