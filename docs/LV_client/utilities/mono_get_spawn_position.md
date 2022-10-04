# mono_get_spawn_position.vi

<p class="img_container">
<img class="lg_img" src="../mono_get_spawn_position.png"/>
</p>

### Description

Obtain the start position and rotation for the vehicle from the available Starting Points.

For technical support contact us at <b>monodrive.support@ni.com</b>  

### Inputs

- **Starting Points:**  Pre-built points on the map where the ego vehicle spawns. Cluster with the following structure:    
   
 Name  | Type  | Description  |
|  |  |  |
|locations  | DBL 2D Array  |x,y.z coordinates |
|rotations | DBL 2D Array  | yaw, pitch, roll|
|type | string  | PlayerStartPIE (camera) or PlayerStart |
 
- **Vehicle Start Position:** Index to select the spawn position and rotation for the ego vehicle

- **monoDrive in (Cluster):** See description at [**monoDrive.ctl**](../structures/monoDrive.md). 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **monoDrive out (Cluster):** See description at [**monoDrive.ctl**](../structures/monoDrive.md). 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
