## mono_start_lane_follower.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/tools/mono__start__lane__followerc.png" 
width="400"/>
</p>

### Description 
This tool is used on the lane follower example. During initialization gets the map on geojson format.

### Inputs
- **Array in:** A reference to the weather profile cluster.
- **Trajectory configuration:** Path to the trajectory configuration file .
- **Select map(Enum):** Name of the map to load.
- **Vehicle Start Position (I32):** 
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **Trajectory configuration(String):** The string of the trajectory read in.
- **weather profiles(Cluster):** A cluster with the weather information.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
