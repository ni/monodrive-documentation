# mono_send_command.vi

<p class="img_container">
<img class="lg_img" src="../mono_send_command.png"/>
</p>

### Description

Send a command via TCP with format "type", "success", "reference", "message". This VI is used to send server commands.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **TCP Network Connection in:**  TCP connection where data is being sent
 

- **JSON Command:**  Command in JSON format.

    ```
        {
          "type": string,
          "success": bool,
          "reference": int,
          "message": JSON,
        }
    ``` 
 

- **Timeout ms (25000):**  Maximum time to wait for a response back from the simulator
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Response:**  Response from the simulator to the message sent.
 

- **Success:**  True if the response from the server was successful to the
command sent. 
 

- **TCP Network Connection Out:**  TCP connection where data is being sent
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
