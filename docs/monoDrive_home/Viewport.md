# Viewport

The Viewport sensor allows a client to configure the camera that is displayed in 
the monoDrive Simulator or Scenario Editor. 

## Configuration

``` json
{
  "type": "ViewportCamera",
  "listen_port": 0,
  "server_ip": "127.0.0.1",
  "server_port":8999,
  "description": "",
  "location": {
    "x":  -800.0,
    "y": 0.0,
    "z": 400.0
    },
  "rotation": {
    "pitch": -15.0,
    "yaw": 0.0,
    "roll": 0.0
    },
  "stream_dimensions": {
    "x": 512.0,
    "y": 512.0
    },
  "monitor_number" :0,
  "window_size": {
    "x": 512.0,
    "y": 512.0
    },
  "window_offset": {
    "x":  0.0,
    "y": 0.0
    },
  "enable_streaming": false,
  "use_vehicle_hud": false,
  "wait_for_fresh_frame": true,
  "sensor_size": 9.069999694824219,
  "fullscreen": false,
  "dynamic_range": 50,
  "fov": 60.0,
  "focal_length": 9.0,
  "fstop": 1.4,
  "max_distance": 50000.0,
  "min_shutter": 0.000500,
  "max_shutter": 0.001400,
  "sensor_size": 9.07,
  "channels": "bgra",
  "channel_depth": 1,
  "annotation": {
    "cull_partial_frame": false,
    "debug_draw": false,
    "desired_tags": [],
    "far_plane": 10000.0,
    "include_annotation": false,
    "include_obb": false,
    "include_tags": false
    }
}
```
- **listen_port:** Any non-zero value will create a secondary viewport. A value of zero is reserved for the main viewport.    
- **monitor_number:** The monitor where the viewport will be created.   
- **window_size**: Window size of the secondary viewport.   
- **window_offset**: An offset in x and y from the left corner of the screen where - the window of secondary viewport will be created.   
- **enable_streaming:** Allow a tcp streamer to work on the same viewport camera.   
- **use_vehicle_hud:** enables the HUD indicators such as speed, gear, etc. to launch any secondary viewport.   
- **fullscreen:** launches the secondary viewport into fullscreen mode on whichever monitor was specified on the **monitor_number:**  tag.   
- **fov:** Sets the horizontal angle of view of the viewport camera.   


### Configuration Examples  

On the left of the image, there are 3 secondary viewport and on the right image there is the main viewport with HUD enable on the monoDrive Editor.

<p class="img_container">
  <img class="wide_img" src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/configuration/viewport/viewport.png" />
</p>
