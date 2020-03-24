## mono_start_replay.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/tools/mono__start__replayc.png" 
width="400"/>
</p>

### Description 
Use this tool to start a Replay application. This takes care of the initialization of the simulator on Replay mode. 

### Inputs
- **Array in:** A reference to the weather profile cluster.
- **Trajectory configuration:** Path to the trajectory configuration file .
- **Select map(Enum):** Name of the map to load.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **Trajectory configuration(String):** The string of the trajectory read in.
- **weather profiles(Cluster):** A cluster with the weather information.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
