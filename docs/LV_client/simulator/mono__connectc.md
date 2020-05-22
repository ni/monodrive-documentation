## mono_connect.vi
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/simulator/mono__connectc.png" />
</p>

### Description 
Open a TCP connection given an IP and control port.

### Inputs

- **Simulator IP (String) :** IP of the simulator, typically **127.0.0.1**.
- **Simulator Port (Int) :** Port to write and read messages from the simualor, typically **8999** .
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **TCP Network connection Out (TCP Network connection) :** TCP connection with the simulator.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>