# mono_send_scenario_config.vi

<p class="img_container">
<img class="lg_img" src="../mono_send_scenario_config.png"/>
</p>

### Description

Configure the trajectory EGO and other cars in the scene will follow  by given a path to JSON file.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **monoDrive in:**  See description at [**monoDrive.ctl**](../structures/monoDrive.md). 

- **Scenario configuration:**  Path to the scenario file to use to run the simulaion on closed loop mode 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **monoDrive out:**   See description at [**monoDrive.ctl**](../structures/monoDrive.md). 

- **Scenario configuration:**  String obtained from reading the file specied on the Scenario Configuration.

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>

