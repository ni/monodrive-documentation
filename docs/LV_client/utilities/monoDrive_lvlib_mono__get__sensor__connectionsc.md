
## mono_get_sensor_connections.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/blob/lv_client_docs/WikiPhotos/LV_client/utilities/monoDrive_lvlib_mono__get__sensor__connectionsc.png?raw=true" 
width="400"  />
</p>

### Description 
Creates an array of clusters for each type of sensor. Each cluster includes port and connection ID.

### Inputs

- **Connections_in(TCP Network connection):** Connection ID corresponding to the TCP connectios of the sensors.
- **Sensors_type (Array of String):** Type name of sensor.
  * "Camera", "Lidar", "GPS", etc...
- **Sensors_ports(Array of Int):** Port number for all the sensors.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **IMU (Array of Cluster) :** Array of clusters, where each cluster correspond to a different IMU sensor.
  - Cluster with 2 elements:
    * connection ID (TCP Network connection): Connection ID corresponding to the TCP connection for this **IMU** sensor.
    * port (int): System port corresponding to this IMU sensor.
- **Radar (Array of Cluster) :** Array of clusters, where each cluster correspond to a different Radar sensor.
    - Cluster with 2 elements:
      * connection ID (TCP Network connection): Connection ID corresponding to the TCP connection for this **Radar** sensor.
      * port (int): System port corresponding to this Radar sensor.
- **Camera (Array of Cluster) :** Array of clusters, where each cluster correspond to a different Camera sensor. 
  - Cluster with 2 elements:
    * connection ID (TCP Network connection): Connection ID corresponding to the TCP connection for this **Camera** sensor.
    * port (int): System port corresponding to this Camera sensor.
- **Lidar (Array of Cluster):** Array of clusters, where each cluster correspond to a different Lidar sensor. 
  - Cluster with 2 elements:
    * connection ID (TCP Network connection): Connection ID corresponding to the TCP connection for this **Lidar** sensor.
    * port (int): System port corresponding to this Lidar sensor.
- **GPS (Array of Cluster):** Array of clusters, where each cluster correspond to a different GPS sensor.
  - Cluster with 2 elements:
    * connection ID (TCP Network connection): Connection ID corresponding to the TCP connection for this **GPS** sensor.
    * port (int): System port corresponding to this GPS sensor.
- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
