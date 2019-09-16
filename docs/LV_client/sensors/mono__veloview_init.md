## mono__veloview_init.vi
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__veloview_init.png"  />
</p>

### Description
Opens a UDP connection to communicate with VeloView program. 

### Inputs

- **veloview address input (String):** Contains the camera information for the **camera** sensor.
- **local veloview tx port (Int):** Local port number, usually 2367.
- **error in (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.


### Outputs
- **veloview UDP connection (UDP Network Connection):** Reference to the UDP Network Connection is opened with this VI.
- **veloview address:** IP address of the machine where veloview is running.
- **error out (Error Cluster):** can accept error information wired from VIs previously called. Use this information to decide if any functionality should be bypassed in the event of errors from other VIs.
