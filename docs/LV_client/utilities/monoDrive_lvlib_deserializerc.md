## deserializer.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/blob/lv_client_docs/WikiPhotos/LV_client/utilities/monoDrive_lvlib_deserializerc.png?raw=true" 
width="400"  />
</p>

### Description 
Parse a string message to remove newline and escape characters. 

### Inputs

- **JSON in:** A JSON format text with extra characters as newline or escape.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **JSON out:** A JSON format text without extra characters as newline or escape.  .
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
