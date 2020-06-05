# mono_gps.vi

<p class="img_container">
<img class="lg_img" src="../mono_gps.png"/>
</p>

### Description

Configure and reads the data stream data for the GPS sensor.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Raw data:**  Raw data output form server. 
 

- **GPS Configuration:**  Settings to configure a GPS sensor
 

- **GPS Sample:**  Provides GPS coordinates for the sensor's location. The
coordinates are based on the GPS anchor set for the current
map      

| Type  | Name   |
| --------- | ------------ |
|U8  | preamble |
|U16 | MSG_POS_LLH  |
|U16 | sensor_id  |
|U8  | payload_length |
|DBL | latitude |
|DBL | longitude  |
|DBL | elevation  |
|SGL | WorldLocation_x |
|SGL | WorldLocation_y|
|SGL | forward_x  |
|SGL | forward_y   |
|SGL | forward_z   |
|SGL | ego_yaw  |
|SGL | ego_speed |
|U16 | horizontal_acceleration  |
|U16 | vertical_acceleration  |
|U8  | satellites  |
|U8  | mode   |
|U16 | CRC  |
--- 

- **latitude:**  Latitude of ego vehicle on the map
 

- **longitude:**  Longitude of ego vehicle on map.
 

- **elevation:**  Elevation of the ego vehicle on the map.
 

- **WorldLocation_x:**  Fix location for the map on the x axis.
 

- **WorldLocation_y:**  Fix location for the map on the y axis
 

- **forward_x:**  Forward (X) unit direction vector, in world space.
 

- **forward_y:**  Forward (Y) unit direction vector, in world space.
 

- **forward_z:**  Forward (Z) unit direction vector, in world space.
 

- **ego_yaw:**  Yaw for the ego vehicle with respect to the Z axis.
 

- **ego_speed:**  Ego's speed measured on m/s
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>

