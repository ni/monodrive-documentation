## mono_connect.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/blob/lv_client_docs/WikiPhotos/LV_client/simulator/monoDrive_lvlib_mono__connectc.png?raw=true" 
width="400"  />
</p>

### Description 
Open a TCP connection given an IP and control port.

### Inputs

- **Simulator IP (String) :** IP of the simulator, typically **127.0.0.1**.
- **Control Port (Int) :** Port to write and read messages from the simualor, typically **8999** .
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **monoDrive Connection (TCP Network connection) :** TCP connection with the simulator.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
