# mono_send_pilot_scenario.vi

<p class="img_container">
<img class="lg_img" src="../mono_send_pilot_scenario.png"/>
</p>

### Description

Reads an scenario file (JSON) and sends it to the server.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **monoDrive in (Cluster):** See description at [**monoDrive.ctl**](../structures/monoDrive.md).  


- **Scenario configuration:**  Path to the scenario file to use to run the simulation on
closed loop mode
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **monoDrive out (Cluster):** See description at [**monoDrive.ctl**](../structures/monoDrive.md).  

- **Scenario configuration:**  Scenario read (JSON)  from the Scenario Configuration path
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>