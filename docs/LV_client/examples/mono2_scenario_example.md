# mono2_scenario_example.vi

<p class="img_container">
<img class="lg_img" src="../mono2_scenario_example.png"/>
</p>

### Description

This example shows how to use the monoDrive client to configure the monoDrive simulator in closed loop mode with all the sensors.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **Throttle:**  Change the vehicle's throttle (0 - 1)
 

- **Steering:**  While driving, move the vehicle right and left (-1 to 1)
 

- **Brake:**  When pressed the vehicle stops (False by default)
 

- **Direction:**  Move the car forward or backward (move forward by default )
 

- **Select a map:**  Select the map to load on the Simulator
 

- **Scenario Configuration:**  Selected a valid scenario file in JSON. Use the browse
button to select a new file.    
User can find **scenario_config_multi_vehicle.json**  under
the monoDrive folder under the **scenarios** folder
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Camera:**  Shows the camera sensor output
 

- **Map:**  Shows a graphic representation of the complete map loaded on
the simulator
 

- **fps:**   

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
