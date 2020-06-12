# mono_header.vi

<p class="img_container">
<img class="lg_img" src="../mono_header.png"/>
</p>

### Description

Parse data from sensor into a cluster given a connection ID.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **TCP Network Connection in:**  TCP connection where data is being sent
 

- **error in (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **TCP Network Connection out:**  TCP connection where data is being sent
 

- **Sensor sample:**  Data sent from the simulator follow the same structure
 

- **Header:**  Information for the data being sent.       

Type  | Name   |
| --------- | ------------ |
|U32  | length |
|SGL | game_time  |
|U32 | time_stamp |
--- 

- **length:**  Length of the data being sent over TCP
 

- **time_stamp:**  Time in milliseconds
 

- **game_time:**  Time since the simulation started

- **frame_count:**  Frame number when this sample was taken
 

- **data out:**  Data sent over TCP
 

- **error out (Error Cluster):** Accepts error information wired from previously called VIs. This information can be used to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
 errors from other VIs. 

<p>&nbsp;</p>
