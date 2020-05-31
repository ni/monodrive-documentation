# mono_header.vi

<p class="img_container">
<img class="lg_img" src="../mono_header.png"/>
</p>

### Description

Parse data from sensor into a cluster, given a connection ID.

For technical support contact us at <b>support@monodrive.io</b> 

### Inputs

- **TCP Network Connection in:**  TCP connection where data is being sent
 

- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs. 

### Outputs

- **Sensor sample:**  Data sent from the simulator follow the same structure
 

- **Header:**  Information for the data being sent.       

Type  | Name   |
| --------- | ------------ |
|U32  | lenght |
|SGL | game_time  |
|U32 | time_stamp |
--- 

- **length:**  Lenght of the data being sent over TCP
 

- **time_stamp:**  Time in milisenconds according to the OS where the simulator
is running
 

- **game_time:**  Time since the client connected to the simulator in
milliseconds 
 

- **data out:**  Data sent over TCP
 

- **TCP Network Connection out:**  TCP connection where data is being sent
 

- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs. 

<p>&nbsp;</p>
