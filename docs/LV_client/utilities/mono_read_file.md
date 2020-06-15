# mono_read_file.vi

<p class="img_container">
<img class="lg_img" src="../mono_read_file.png"/>
</p>

### Description

Read the trajectory specified from the name or absolute path.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **File path in:**  Path to the trajectory or scenario file, this can be an
absolute or relative path
 

- **Base directory name:**  Name of the directory where the trajectories or scenarios
are located.    
    - trajectories: Files used on replay mode.
    - scenarios: Files used on closed loop mode.
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Trajectory or Scenario read:**  Trajectory or scenario string (JSON format) obtained from
reading the file specified on  the **File path in**
 

- **File path out:**  Path to the trajectory or scenario file.
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
