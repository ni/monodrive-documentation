# mono_replace_ego_spawn_position.vi

<p class="img_container">
<img class="lg_img" src="../mono_replace_ego_spawn_position.png"/>
</p>

### Description

Replace the start position and pose of the ego vehicle using the information from the starting position selected by the user 

### Inputs

- **Vehicles in:**  A list with all the vehicles
 

- **Start location:**  Coordinates (x,y,z) for the start position
 

- **Vehicle settings:**  Settings all vehicles have, ie. body color, body type, etc
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Vehicles out:**  The list of the vehicles including the modified ego vehicle
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
