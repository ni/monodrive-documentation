# mono_send_step.vi

<p class="img_container">
<img class="lg_img" src="../mono_send_step.png"/>
</p>

### Description

Send the command to step the simulation for a given amount. Specify the desired amount in the message string.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **Command_ID:**  Command to move one step in simulation.  Valid message
**REPLAY_StepSimulationCommand_ID**
 

- **message:**  A string in the following format where the number specifies
how many steps to move in simulation.   

```
{"amount":1}
```   
 

- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
