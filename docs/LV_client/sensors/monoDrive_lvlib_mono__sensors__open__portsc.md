## mono_sensors_open_ports.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/blob/lv_client_docs/WikiPhotos/LV_client/sensors/monoDrive_lvlib_mono__sensors__open__portsc.png?raw=true" 
width="400"  />
</p>

## Description
Open the ports for all the sensors configured, by getting each port in an array of ports. Returns an array with the connection ID corresponding to each port opened.
### Inputs

- **Simulator IP(String):**  IP of the simulator, typically **127.0.0.1**.
- **Ports(Array of int):** Array with all the ports numbers for sensors specified in the Sensor config file.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **Connections(Array of TCP Network connection):** Array with all the TCP connections opened for sensors specified in the Sensor config file.
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
