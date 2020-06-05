# mono1_pilot_example.vi

<p class="img_container">
<img class="lg_img" src="../mono1_pilot_example.png"/>
</p>

### Description

This example shows how to use the monoDrive client to configure the monoDrive simulator in closed loop mode with all the sensors.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **Throttle:**  Change the vehicle's throttle (0 - 1)
 

- **Steering:**  While driving, move the vehicle right and left (-1 to 1)
 

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

- **Camera:**  Shows the camera sensor output
 

- **Map:**  Shows a graphic representation of the complete map loaded on
the simulator
 

- **fps:**  Frames per Second at which the client is running
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>

