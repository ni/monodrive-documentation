## Multi viewport

For many high performance use cases, TCP is simply not fast enough to stream the large amount of camera data
we are simulating in real time. As a better performance alternative, monoDrive supports streaming camera data
directly from the GPU over display output.

The user can configure a viewport on any standard, fisheye, or 360 camera. This will open
an additional window on the simulation machine to directly display the stream of camera data.

### Usage

Configure a camera with a viewport:

```json
  {
    "type": "Camera",
    "listen_port": 8001,
    "enable_streaming": false,
    "location": {
      "x": 0.0,
      "y": -25.0,
      "z": 225.0
    },
    "rotation": {
      "pitch": -5.0,
      "yaw": -15.0,
      "roll": 0.0
    },
    "stream_dimensions": {
      "x": 512.0,
      "y": 512.0
    },
    "max_distance": 50000.0,
    "dynamic_range": 50,
    "fov": 60.0,
    "focal_length": 9.0,
    "fstop": 1.4,
    "min_shutter": 0.000500,
    "max_shutter": 0.001400,
    "sensor_size": 9.07,
    "channels": "bgra",
    "viewport": {
      "enable_viewport": true,
      "monitor_name": "",
      "monitor_number": 0,
      "fullscreen": false,
      "window_offset": {
        "x": 256.0,
        "y": 256.0
      }
    }
  }
```

- **enable_streaming:** Should usually be set to `false` to disable TCP streaming for performance. This can be left on
if TCP streaming is also desired.
- **stream_dimensions:** The size of the image in pixels
    - **x:** The width of the image in pixels
    - **y:** The height of the image in pixels
- **viewport:** Viewport configuration
    - **enable_viewport:** If `true`, will enable camera viewport
    - **monitor_name:** If specified, will position viewport on specific monitor by name
    - **monitor_number:** If specified, will position viewport on specific monitor by index. Note that if `monitor_name`
    is also specified, this index will be within the matching group of monitors.
    - **fullscreen:** If `true`, the camera viewport will be set to borderless fullscreen
    - **window_offset:** The viewport window offset from top-left of monitor. This only applies if `fullscreen` is `false`.
        - **x:** The window offset in x
        - **y:** The window offset in y


### Example
<p class="img_container">
<img class="lg_img" src="../img/multiviewport.png"/>
</p>


