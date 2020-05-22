## mono_send_vehicle_command.vi
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/utilities/mono__send__vehicle__commandc.png" 
  />
</p>

### Description 
Sends a command to the monoDrive Simulator to control the movement of ego vehicle.

### Inputs

- **monoDrive in (Cluster):** See description at **monoDrive.ctl**.
- **forward_amount:** a value between -1.0 and 1.0 that determines the amount of throttle to apply.
- **right_amount :** a value between -1.0 and 1.0 that determines the steering position (-1 is left, +1 is right).
- **drive_mode (Boolean):** Move forward or backwards.
- **brake (Boolean):** Enable and disable Brake.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>