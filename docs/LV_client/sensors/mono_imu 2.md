# mono_imu.vi

<p class="img_container">
<img class="lg_img" src="../mono_imu.png"/>
</p>

### Description

Reads and process the data stream for the IMU sensor. Outputs the IMU data in a cluster.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **IMU Sample:**  Cluster with the processed data for the IMU sensor.   

| Type  | Name   |
| --------- | ------------ |
|U8  | packetSize |s
|SGL | accel_x  |
|SGL | accel_x  |
|SGL | accel_z |
|SGL | angle_rate_x |
|SGL | angle_rate_y  |
|SGL | angle_rate_z  |
|U32 | timer |
|U16 | checksum|
|I32 | timeWeek  |
--- 

- **packetSize:**  Start Byte
 

- **accel_x:**  The  acceleration in the x direction 
 

- **accel_y:**  The  acceleration in the y direction 
 

- **accel_z:**  The  acceleration in the z direction 
 

- **angle_rate_x:**  The angular velocity in th xdirection 
 

- **angle_rate_y:**  The angular velocity in th y direction 
 

- **angle_rate_z:**  The angular velocity in th z direction 
 

- **timer:**  Time in seconds since the system power-up.
 

- **checksum:**  Checksum
 

- **timeWeek:**  Time of the week
 

- **Raw data:**  Output with out parsing  from the simulator.
 

- **IMU configuration:**  Configuration used to setup the IMU sensors.
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
