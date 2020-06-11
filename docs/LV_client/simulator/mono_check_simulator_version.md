# mono_check_simulator_version.vi

<p class="img_container">
<img class="lg_img" src="../mono_check_simulator_version.png"/>
</p>

### Description

Check compatibility between the simulator's version and the client's version.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **Response JSON:**  Response from the simulator including the information for
the simulator version 
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Simulator Version:**  Simulator version number as an array of numbers, obtained
from the server response
 

- **Compatible:**  True if the simulator and client are compatible based on the
API versions
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
p>
