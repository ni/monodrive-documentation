## mono_header.vi
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/simulator/mono__headerc.png" />
</p>

### Description 
Parse data from sensor into a cluster, given a TCP connection ID.

### Inputs

- **TCP Network connection in (TCP Network connection):** TCP connection ID of a sensor.
- **error in (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs

- **TCP Network connection out (TCP Network connection):** TCP connection ID same as input connection ID  .
- **sensor_sample(Cluster of cluster):** Obtain header information and raw data.

| Cluster        | 
| ------------- | 
| <ul><li>lenght(U32)</li><li>time_stamp(U32)</li><li>game_time(SGL)</li></ul>|
| data out (String) |

- **error out (Error Cluster):** Can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.

<p>&nbsp;</p>