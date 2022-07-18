# mono_check_config_mode.vi

<p class="img_container">
<img class="lg_img" src="../mono_set_running_mode.png"/>
</p>

### Description

Builds a string using the simulator configuration and using a running mode chosen by the user.

For technical support contact us at <b>monodrive.support@ni.com</b> 

### Inputs

- **Configuration in:**  Simulator configuration in JSON format
 

- **Running mode:**  Configure the running mode for the simulator:   

    - 0 - Closed_loop    
    - 1 - Replay     
    - 2 - Replay_step
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Configuration out:**  JSON configuration built using the simulator configuration
and the Running mode selected.
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
