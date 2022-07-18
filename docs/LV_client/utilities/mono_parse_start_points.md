# mono_parse_start_points.vi

<p class="img_container">
<img class="lg_img" src="../mono_parse_start_points.png"/>
</p>

### Description

Parse the starting point information and generate a cluster with the coordinates of each point and the type. If a point is PIE type, it will be move to the top of the list. The PIE point is the camera position.

For technical support contact us at <b>monodrive.support@ni.com</b> 

### Inputs

- **Response:**  Response from the server to the **GetStartPoints** command 
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Start locations:**  Pre-built points on the map where the ego vehicle can be
spawn. Cluster with the following structure:   

Name  | Type  | Description  |
|  |  |  |
|locations  | DBL 2D Array  |x,y.z coordinates |
|rotations | DBL 2D Array  | yaw, pitch, roll|
|type | string  | PlayerStartPIE (camera) or PlayerStart |
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
