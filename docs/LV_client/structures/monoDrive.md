## monoDrive.ctl
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/structures/monoDrivectlc.png" width="50"  />
</p>

### Description 
The monoDrive control, contains configuration parameters and variables to be used and updated by the client.

### Structure
- **Config  (Cluster):** Configuration parameters for Trajectory and Weather .
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/structures/configctlc.png" width="50"  />
</p>

  **Trajectory**

| Type  | Name   |
| ------------ | ------------ |
|I32  | Number of trajectories |
|I32 | Trajectory Number  |
|String | Trajectory  |

  **Weather**

| Type  | Name   |
| ------------ | ------------ |
|String  | id|


- **Connections (Cluster):** Configuration parameters for the simulator .
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/structures/connectionsctlc.png" width="50"  />
</p>

| Type  | Name   |
| ------------ | ------------ |
|TCP connection  | Simulator Connection |
|String | IP Address  |


