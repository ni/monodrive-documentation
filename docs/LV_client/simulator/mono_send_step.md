# mono_send_step.vi

<p class="img_container">
<img class="lg_img" src="../mono_send_step.png"/>
</p>

### Description

Send the command to step the simulation for any given amount. Specify the desired amount in the message string.

For technical support contact us at <b>monodrive.support@ni.com</b> 

### Inputs

- **Command_ID:**  Server command.    
 **REPLAY_StepSimulationCommand_ID**   
 **REPLAY_StateStepSimulationCommand_ID**
 

- **message:**  A string in the following format where the number specifies
how many steps to move in simulation.    

    ```
        {"amount":1}
    ```   
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
