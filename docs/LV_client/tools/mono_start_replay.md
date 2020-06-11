# mono_start_replay.vi

<p class="img_container">
<img class="lg_img" src="../mono_start_replay.png"/>
</p>

### Description

Tool for initialization of a Replay mode example.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **Trajectory Configuration:**  Path to a replay file (JSON format) of a previously recorded trajectory  using the monoDrive editor.

- **Weather Profile Ref:** Reference to the Weather Profile Array  

- **Select a map:**  Select a map to load on Simulator. 

- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Trajectory configuration:** String read from the trajectory file

- **weather profiles:**  Array of the weather profiles obtained by the weather configuration.



- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
