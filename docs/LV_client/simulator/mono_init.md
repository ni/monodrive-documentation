# monoDrive.lvlib:mono_init.vi

<p class="img_container">
<img class="lg_img" src="../mono_init.png"/>
</p>

### Description 
Module to initialize the monoDrive simulator. This VI must be run before other simulator configuration/vehicle commands are run.


### Inputs
- **Simulator configuration (String):** A JSON string with the properties for simulator connection, material properties, etc.

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **monoDrive out (Cluster):** See description at monoDrive.ctl.

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>