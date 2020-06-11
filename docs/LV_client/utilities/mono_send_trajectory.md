# mono_send_trajectory.vi

<p class="img_container">
<img class="lg_img" src="../mono_send_trajectory.png"/>
</p>

### Description

Configure the trajectory EGO and other cars in the scene will follow  by given a path to JSON file.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **monoDrive in (Cluster):** See description at [**monoDrive.ctl**](../structures/monoDrive.md).  


- **Trajectory Configuration:**  Path to the trajectory configuration (JSON)
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **monoDrive out (Cluster):** See description at [**monoDrive.ctl**](../structures/monoDrive.md). 

- **Trajectory configuration:** Trajectory read (JSON) from the specified path on the
Trajectory Configuration

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
