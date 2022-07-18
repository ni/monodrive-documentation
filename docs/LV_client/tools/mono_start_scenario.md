# mono_start_scenario.vi

<p class="img_container">
<img class="lg_img" src="../mono_start_scenario.png"/>
</p>

### Description

Tool for initialization of a Closed loop example.

For technical support contact us at <b>monodrive.support@ni.com</b> 

### Inputs

- **Weather Profile Ref:**   Reference to the Weather Profile Array 

- **Select a map:** Select a map to load on Simulator.  

- **Scenario Configuration:** Path to the a scenario file (JSON format) previously recorded using the monoDrive scenario tool.  

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Weather Profiles:**  Array of the weather profiles obtained by the weather configuration. 


- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
