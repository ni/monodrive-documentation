## mono_sensor_parser.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/blob/lv_client_docs/WikiPhotos/LV_client/utilities/monoDrive_lvlib_mono__sensor__parserc.png?raw=true" 
width="400"  />
</p>

### Description 
Gets Sensor type and ports from the configuration text.

### Inputs

- **Sensor_config (String):** Text obtained from reading the Sensor configuration file.
- **Type (String):** Tag used in the Sensor configuration file: **"type"**
- **Port (String):** Tag used in the Sensor configuration file: **"listen_port"**
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **Sensor (String):** Type name of sensor  .
- **Remain_string (String):** To be used with shift register. Remainding text.
- **Port out (Int):** Port obtained .
- **EOL (Boolean):** End of the file .
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
