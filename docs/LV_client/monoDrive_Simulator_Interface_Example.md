## monoDrive Simulator Interface Example.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/monoDrive_Simulator_Interface_Examplec.png" 
width="400"  />
</p>

### Inputs

- **Control Port (int):** Port to write and read messages from the simualor, typically **8999**.
- **Simulator IP (string):** IP of the simulator, typically **127.0.0.1**.
- **Simulator Config (Path):** Path or name of the file with the Simulator configuration in JSON format.
- **Sensor Config (Path):** Path or name of the file with the Sensor configuration in JSON format.
- **Trajectory Config (Path):** Path or name of the file with the Trajectory configuration in JSON format.
- **Simulator Error In (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **Step (boolean):** Indicate if the simulator is runnig on replay or replay_step mode.
- **Driving (boolean):** Indicate if the simulator is runnig on closed_loop mode.
