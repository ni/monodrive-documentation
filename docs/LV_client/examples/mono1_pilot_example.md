# mono1_pilot_example.vi

<p class="img_container">
<img class="lg_img" src="../mono1_pilot_example.png"/>
</p>

### Description

This example shows how to use the monoDrive client to configure the monoDrive simulator in closed loop mode with all the sensors.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **Throttle:**  Change the vehicle's throttle (0 - 1)
 

- **Steering:**  Move to drive the car right and left (-1 to 1)
 

- **Brake:**  When pressed the vehicle stops (False by default)
 

- **Direction:**  Move the car forward or backward (move forward by default )
 

- **Vehicle  Start Position:**  Select a position from the pre-defined initial position to
spawn the ego vehicle.   
    - Selenct **-1** to spawn the vehicle on the position of
the camera.    
     - Select **0** to spawn the vehicke on the position
specified on the Closed_loop.json file
 

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
 

- **Map:**  Shows a graphic representation of the complete map loaded on
the simulator
 

- **fps:**  Frames per Second at which the client is running
 

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
