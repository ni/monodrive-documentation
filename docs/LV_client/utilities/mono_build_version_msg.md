# mono_build_version_msg.vi

<p class="img_container">
<img class="lg_img" src="../mono_build_version_msg.png"/>
</p>

### Description

Builds a message to inform the user the compatibility between simulator and client.

For technical support contact us at <b>monodrive.support@ni.com</b> 

### Inputs

- **Client version:**  Current client version
 

- **Client API version:**  Current client API Version
 

- **Response JSON:**  Response from the simulator including the server version and
the sever API version.   

```json
    {
        "type": string,
        "success": bool,
        "reference": int,
        "message": JSON
    }
```
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Message:**  Output a message with information regarding the compatility
of the simulator and client
 

- **API version:**  Current API version of the simulator
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
