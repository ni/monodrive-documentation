# mono_send_vehicle_command.vi

<p class="img_container">
<img class="lg_img" src="../build_manual_control.png"/>
</p>

### Description
Tool to build a message to control the ego vehicle. Allows to change from autopilot to manual control. 


### Inputs

- **Vehicle Control:** Allows control of the ego vehicle using steering angle, throttle, brake and direction values.
- **set_manual_control_mode_pedal:** enables to use manual control of the pedal.
- **manual_gear:** Allows to control the ego vehicle using a specifict value for the gear.



### Outputs
- **manual_control:** The set of controls needed for manual control of the ego vehicle.
- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
