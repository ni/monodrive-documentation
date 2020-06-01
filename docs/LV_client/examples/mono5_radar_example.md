# mono5_radar_example.vi

<p class="img_container">
<img class="lg_img" src="../mono5_radar_example.png"/>
</p>

### Description

This example shows how to run the monoDrive client  to configure the monoDrive simulator on Replay with a Radar sensor. This example provides a tool to plot Radar and Ground Truth information. 

### Inputs

- **Trajectory Configuration:**  Selected a valid scenario file in JSON. Use the browse
button to select a new file. User can find
**HighWayExitReplay.json** under the monoDrive folder under
the trajectories folder
 

- **Select a map:**  Select the map to load on the Simulator
 

- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Status:**  error in can accept error information wired from VIs
previously called. Use this information to decide if any
functionality should be bypassed in the event of errors from
other VIs.

Right-click the error in control on the front panel and
select Explain Error or Explain Warning from the shortcut
menu for more information about the error.
 

- **Camera:**  Shows the camera sensor output
 

- **Number of frames:**  umber of frames found on the Trajectory read
 

- **fps:**  Frames per Second at which the client is running
 

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
