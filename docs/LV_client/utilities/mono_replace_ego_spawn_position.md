# mono_replace_ego_spawn_position.vi

<p class="img_container">
<img class="lg_img" src="../mono_replace_ego_spawn_position.png"/>
</p>

### Description

Replace the start position and pose of the ego vehicle using the information from the starting position selected by the user 

### Inputs

- **Start rotation:**  Rotation information (yaw,pitch, roll) for the start point
- **Vehicles in:** List of the vehicles in the scenario.

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Vehicles out:**  List of the vehicles including the modified ego vehicle
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
