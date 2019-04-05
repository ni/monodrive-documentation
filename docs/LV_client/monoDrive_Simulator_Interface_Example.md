## monoDrive Simulator Interface Example.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/monoDrive_Simulator_Interface_Examplec.png" 
width="400"  />
</p>

### Inputs

- **TestStant (Boolean):** Using TestStand to run the client.
- **Running Mode(string):** Indicate mode of operation. Closed_loop, Replay or Replay_step.
- **Simulator Config (Path):** Path or name of the file with the Simulator configuration in JSON format.
- **Sensor Config (Path):** Path or name of the file with the Sensor configuration in JSON format.
- **Trajectory Config (Path):** Path or name of the file with the Trajectory configuration in JSON format.
- **Ok settings (Boolean):** To use with Test Stand, activate configuration.
- **Weather Index (Int):** Weather configuration. To use with Test Stand or external VI.
- **error In (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **Step (boolean):** Indicate if the simulator is runnig on replay or replay_step mode.
- **Driving (boolean):** Indicate if the simulator is runnig on closed_loop mode.
