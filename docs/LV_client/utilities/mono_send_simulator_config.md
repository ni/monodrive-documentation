# mono_send_simulator_config.vi

<p class="img_container">
<img class="lg_img" src="../mono_send_simulator_config.png"/>
</p>

### Description

Configures simulator given a path to a  JSON file. The file will configure server ip, port, etc.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **monoDrive in (Cluster):** See description at [**monoDrive.ctl**](../structures/monoDrive.md).  

- **Simulator Configuration:**  Simulator configuration (JSON string) 
 
- **Running mode:**  Configure the running mode for the simulator:    

    - 0 - Closed_loop
    - 1 - Replay
    - 2 - Replay_step
 

- **Select a map:**  Select the map to load on the Simulator
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **monoDrive out (Cluster):** See description at [**monoDrive.ctl**](../structures/monoDrive.md). 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
