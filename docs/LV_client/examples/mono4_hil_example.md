# mono4_hil_example.vi

<p class="img_container">
<img class="lg_img" src="../mono4_hil_example.png"/>
</p>

### Description

This example shows how to run the monoDrive client  to configure the monoDrive simulator on HIL mode.
For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **Trajectory Configuration:**  Select a valid replay file. To change, use the browse button to select a new file. User can find an example by navigating to the monoDrive folder >> trajectories folder >> HighWayExitReplay.json.

- **Select a map:**  Select the map to load on the Simulator

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Camera:**  Shows the camera sensor output
 

- **fps:**  Frames per Second at which the client is running
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
