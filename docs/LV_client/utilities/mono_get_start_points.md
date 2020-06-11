# mono_get_start_points.vi

<p class="img_container">
<img class="lg_img" src="../mono_get_start_points.png"/>
</p>

### Description

Send the GetStartPoints command to obtain the starting points for the map.s

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **monoDrive in (Cluster):** See description at [**monoDrive.ctl**](../structures/monoDrive.md). 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Starting Points:**  Pre-built points on the map where the ego vehicle can be
spawn. Cluster with the following structure:   

 Name  | Type  | Description  |
|  |  |  |
|locations  | DBL 2D Array  |x,y.z coordinates |
|rotations | DBL 2D Array  | yaw, pitch, roll|
|type | string  | PlayerStartPIE (camera) or PlayerStart |
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
