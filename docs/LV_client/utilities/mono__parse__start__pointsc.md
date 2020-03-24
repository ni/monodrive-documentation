## mono__parse__start__points.vi
<p align="center">
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/utilities/mono__parse__start__pointsc.png" 
width="400"  />
</p>

### Description 
Convert the response from the server to the GetStartPoints to a cluster with the three 1D array for `locations`, `rotations` and `type`.


### Inputs
- **response(String):** Response from the server withe `locations`, `rotations` and `type` for each starting point  .
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **start_locations :** A cluster with the three 1D array for `locations`, `rotations` and `type`  .
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
