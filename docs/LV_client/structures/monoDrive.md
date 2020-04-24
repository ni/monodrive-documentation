# monoDrive.ctl

<p class="img_container">
<img class="thumbnail" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/structures/monoDrivectlc.png"/>
</p>

## Description 
The monoDrive control, contains configuration parameters and variables to be used and updated by the client.

## Structure

**Config (Cluster):** Configuration parameters for Trajectory and Weather.

<p class="img_container">
<img class="thumbnail" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/structures/configctlc.png"/>
</p>

  - Trajectory

  | Type  | Name   |
  | ------------ | ------------ |
  |I32  | Number of trajectories |
  |I32 | Trajectory Number  |
  |String | Trajectory  |

  - Weather

  | Type  | Name   |
  | ------------ | ------------ |
  |String  | id|

<p class="img_container">&nbsp;</p>

**Connections (Cluster):** Configuration parameters for the Simulator.

<p class="img_container">
<img class="thumbnail" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/structures/connectionsctlc.png"/>
</p>

  - Simulator 

  | Type  | Name   |
  | ------------ | ------------ |
  |TCP connection  | Simulator Connection |
  |String | IP Address  |
