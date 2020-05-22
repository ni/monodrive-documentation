## mono_remove_response_message.vi
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/utilities/mono__remove__response__messagec.png"   />
</p>

### Description 
Remove curly braces from the "message" section of the simulator's response

### Inputs

- **response_json (String):** Response of the server.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
 
- **json_generated (String):** Response of the server after removing curly braces in the "message"section.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>