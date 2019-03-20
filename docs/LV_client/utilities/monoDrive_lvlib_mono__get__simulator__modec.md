## mono_get_simulator_mode.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/blob/lv_client_docs/WikiPhotos/LV_client/utilities/monoDrive_lvlib_mono__get__simulator__modec.png?raw=true" 
width="400"  />
</p>

### Description 

### Inputs

- **congig_text (String):** Text obtained from the simulator config file .
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **Mode (Int):** Mode set in the simulator config file i.e. "simulation_mode": 1 this VI will return 1.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
