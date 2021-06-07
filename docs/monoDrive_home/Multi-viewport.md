## Multi viewport

For many high performance use cases, TCP is simply not fast enough to stream the large amount of camera data
we are simulating in real time. As a better performance alternative, monoDrive supports streaming camera data
directly from the GPU over display output.

The user can configure a viewport on any standard, fisheye, or 360 camera. This will open
an additional window on the simulation machine to directly display the stream of camera data.

### Usage

Configure a camera with a viewport:

``` json
{
  "type": "Camera",
  "listen_port": 8010,
  "location": {
    "x": 0.0,
    "y": 0.0,
    "z": 250.0
  },
  "rotation": {
    "pitch": 0.0,
    "roll": 0.0,
    "yaw": 0.0
  },
  "stream_dimensions": {
    "x": 512,
    "y": 512
  },
  "annotation": {
    "cull_partial_frame": false,
    "cull_partial_frame_min": 1.0,
    "debug_draw": false,
    "desired_tags": [
      "traffic_sign", "vehicle"
    ],
    "far_plane": 10000.0,
    "include_annotation": false,
    "include_lanes": true,
    "include_obb": false,
    "include_tags": true,
    "lane_sampling_distance": 10000.0,
    "lane_sampling_frequency": 100.0,
    "lane_subsampling": 10.0
  },
  "camera_matrix": [
    [443.0, 0.0, 256.0],
    [0.0, 443.0, 256.0],
    [0.0, 0.0, 1.0]
  ],
  "channel_depth": 1,
  "channels": "bgra",
  "color_filter_array": {
    "cfa": "rccc",
    "use_cfa": false
  },
  "dynamic_range": 1.0,
  "exposure_compensation": -1.0,
  "fov": 60.0,
  "image_noise_bias": 0.0,
  "min_shutter": 0.0005000000237487257,
  "max_shutter": 0.00139999995008111,
  "motion_blur_bias": 0.10000000149011612,
  "white_balance": 0.0,
  "white_balance_temp_offset": 0.0,
  "ray_tracing": {
    "enable_ray_tracing": false
  }
  "viewport": {
    "enable_hud": false,
    "enable_viewport": false,
    "fullscreen": false,
    "hud_class_path": "WidgetBlueprint'/Game/HUD/VehicleHUD.VehicleHUD_C'",
    "monitor_name": "",
    "monitor_number": 0,
    "window_offset": {
      "x": 32,
      "y": 32
    },
    "window_size": {
      "x": 0,
      "y": 0
    }
  }
}
```

- **enable_streaming:** Should usually be set to `false` to disable TCP streaming for performance. This can be left on
if TCP streaming is also desired.
- **stream_dimensions:** The size of the image in pixels.
    - **x:** The width of the image in pixels.
    - **y:** The height of the image in pixels.
- **viewport:** Viewport configuration.
    - **enable_viewport:** If `true`, will enable camera viewport.
    - **enable_hud:** If `true`, will enable HUD (Heads Up Display).
    - **hud_class_path:** Path to HUD Blueprint.
    - **monitor_name:** If specified, will position viewport on specific monitor by name.
    - **monitor_number:** If specified, will position viewport on specific monitor by index. Note that if `monitor_name`.
    is also specified, this index will be within the matching group of monitors.
    - **fullscreen:** If `true`, the camera viewport will be set to borderless fullscreen.
    - **window_offset:** The viewport window offset from top-left of monitor. This only applies if `fullscreen` is `false`.
        - **x:** The window offset in x.
        - **y:** The window offset in y.
    -**window_size:** 
        - **x:** The window offset in x.
        - **y:** The window offset in y.


### Example
<p class="img_container">
<img class="lg_img" src="../img/multiviewport.png"/>
</p>


