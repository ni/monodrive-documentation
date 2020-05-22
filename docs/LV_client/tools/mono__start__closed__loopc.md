## mono_start_closed_loop.vi
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/tools/mono__start__closed__loopc.png" />
</p>

### Description 
Use this tool to start a Closed Loop  application. This takes care of the initialization of the simulator on Closed Loop mode. 

### Inputs
- **Array in(Reference):** A reference to the weather profile cluster.
- **Trajectory configuration(Path):** Path to the trajectory configuration file .
- **Select map(Enum):** Name of the map to load.
- **Vehicle Start Position(I32):** Position where the vehicle will be spawn from the start points on the map.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **Trajectory configuration(String):** The string of the trajectory read in.
- **weather profiles(Cluster):** A cluster with the weather information.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>
