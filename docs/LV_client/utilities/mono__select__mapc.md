## mono_select_map.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/utilities/mono__select__mapc.png" width="400"  />
</p>

### Description 
Builds a simulator using the name specified.

### Inputs

- **Simulator configuration in (String):** A string with the complete simulartor configuration.
- **Select map (String):** The name of the map to use for simulation i.e. Almono_P, Straightaway5k,Straightaway5k_empty, etc.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **Simulator configuration out (String):** The modified simulator configuration, according to the selected map.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
