## mono_send_trajectory.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/mono__send__trajectoryc.png" 
width="400"  />
</p>

### Description 
Configure the trajectory EGO and other cars in the scene will follow  by given a path to JSON file.

### Inputs

- **monoDrive in (Cluster):** See description at **monoDrive.ctl**.
- **Trajectory Config (Path):** Path or name of the file with the Sensor configuration in JSON format.

### Outputs

- **monoDrive out (Cluster):** See description at **monoDrive.ctl**.
- **Trajectory configuration (String) :** Trajectory configuration read from the specified path.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
