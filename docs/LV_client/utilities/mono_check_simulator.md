# mono_check_simulator.vi

<p class="img_container">
<img class="lg_img" src="../mono_check_simulator.png"/>
</p>

### Description

Check if the simulator is open and running. If it is not running then it will stop running the client.

For technical support contact us at <b>monodrive.support@ni.com</b> 

### Inputs

- **monoDrive in (Cluster):** See description at [**monoDrive.ctl**](../structures/monoDrive.md). 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Connected:**  True if the simulator is running, false otherwise
 
- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
