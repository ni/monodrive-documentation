# mono_collision.vi

<p class="img_container">
<img class="lg_img" src="../mono_collision.png"/>
</p>

### Description

Configure and sample the collision sensor. Outputs information on the Time to collision, among other relevant information to detect a collision with vehicles or objects with the tag "vt" 

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Collision sensor sample:**  Sample from collision sensor. 
 

- **Collision sensor configuration:**  Configuration used for sensor collision.
 

- **Accumulative sensor sample:**  Array to accumulate the collision sensor output for each
frame
 

- **sample:**  Collision information for one frame
 

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
