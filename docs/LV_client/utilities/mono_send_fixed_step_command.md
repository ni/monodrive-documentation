# mono_send_fixed_step_command.vi

<p class="img_container">
<img class="lg_img" src="../mono_send_fixed_step_command.png"/>
</p>

### Description

Sends a set of commands to the monoDrive Simulator to control the movement of ego vehicle.    

    - ClosedLoopStepCommand_ID
    - SampleSensorsCommand_ID
    - EgoControl_ID


For technical support contact us at <b>monodrive.support@ni.com</b> 

### Inputs

- **Right amount:**  Value between -1.0 and 1.0 that determines the steering
position (-1 is left, +1 is right)
 

- **Forward amount:**  Value between -1.0 and 1.0 that determines the amount of
throttle to apply
 

- **Brake:**  True to make the car stop.
 

- **Drive mode:**  False to drive the car forward
 

- **Message:**  Send a message with the specified time step to run the
simulation
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
