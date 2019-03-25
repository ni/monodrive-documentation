## mono_get_vehicles.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/blob/master/WikiPhotos/LV_client/utilities/monoDrive_lvlib_mono__get__vehiclesc.png" 
width="400"  />
</p>

### Description 
Get the name of the vehicles and attributes of each trajectory in JSON format given an index.

### Inputs

- **Index (Int):** The number of trajectory to index from **AllTrajectories**. Number of current iteration.
- **All Trajectories (Invariant) :** All trajectories obtained from the trajectory config file as a JSON object in LabVIEW.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **AllVehicles (Invariant):** All vehicles obtained from the trajectory object as a JSON object in LabVIEW. .
- **EOL (Boolean):** True when it reaches the end of the trajectories.
- **Names (Array of String):** An array with the names of the Vehicles in the trajectory.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
