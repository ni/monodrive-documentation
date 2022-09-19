# mono_test_FPS_by_sensor.vi

<p class="img_container">
<img class="lg_img" src="../mono_test_FPS_by_sensor.png"/>
</p>

### Description

This tools is used to measure the FPS for one sensor at the time.

For technical support contact us at <b>monodrive.support@ni.com</b> 

### Inputs

- **Throttle:**  Change the vehicle's throttle (0 - 1)
 

- **Steering:**  Move to drive the car right and left (-1 to 1)
 

- **Brake:**  When pressed the vehicle stops (False by default)
 

- **Direction:**  Move the car forward or backward (move forward by default )
 

- **Vehicle  Start Position:**  Select a position from the pre-defined initial position to
spawn the ego vehicle.   
    - Select **-1** to spawn the vehicle on the position of
the camera.    
     - Select **0** to spawn the vehicle on the position
specified on the Closed_loop.json file
 

- **Select a map:**  Select the map to load on the Simulator
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Status:**  error in can accept error information wired from VIs
previously called. Use this information to decide if any
functionality should be bypassed in the event of errors from
other VIs.

Right-click the error in control on the front panel and
select Explain Error or Explain Warning from the shortcut
menu for more information about the error.
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
