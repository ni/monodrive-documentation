## mono_check_simulator.vi
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/mono__check__simulatorc.png"   />
</p>

### Description 
Check if the simulator is open and running. If it is not running then it will stop running the client.

### Inputs
- **monoDrive in (Cluster):** See description at **monoDrive.ctl**.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **Connected (Boolean) :** True if the simulator is running.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>