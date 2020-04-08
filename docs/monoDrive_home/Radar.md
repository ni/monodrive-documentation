# Radar Sensor

## Radar
Provides radar stream.

```
[
    {
  "type": "Radar",
   "gpu_number":0,
  "listen_port": 8302,
"send_radar_cube": false,
  "paint_targets": false,
  "target_paint_lifetime": 0.5,
  "location": {
    "x": 290.0,
    "y": 0.0,
    "z": 80.0
  },
  "rotation": {
    "pitch": 0.0,
    "yaw": 0.0,
    "roll": 0.0
  },
  "num_samples_per_sweep":345,
  "fs": 50000000,
  "fc": 77000000000.0,
  "num_sweeps": 32,
  "sweep_time" : 0.0000069,
  "bandwidth" : 250000000,
  "max_radar_returns": 500,
  "elements": 8,
  "transmitter": {
    "peak_power": 5.0,
    "aperture": 0.000859,
    "gain": 13.50
  },
  "receiver": {
    "aperture": 0.000798,
    "nf": 10.0,
    "noise_temp": 290.0,
    "nb": 74000000.0,
    "gain": 10.0
  },
  "sbr": {
    "long_range_scan_distance": 60.0,
    "short_range_scan_distance": 30.0,
    "long_range_fov": 60.0,
    "short_range_fov": 30.0,
    "elevation_fov": 10.0,
    "ray_division_y": 10.0,
    "ray_division_z":10.0,
    "debug_frustum": false,
    "debug_scan": false,
    "debug_rescan": false
  }
}
]
```
<p>&nbsp;</p>



## Configuration

### Configuration Tags

- **gpu_number:** Select the specific GPU number(0-N) where each radar sensor will run.
- **send_radar_cube:** Enable to obtain the radar cube from the simulator. 
- **paint_targets:** Tool to draw a box around the targets detected.
- **target_paint_lifetime:** Specify how long a painted target box will be rendered for in the engine, in seconds.
- **location:** Modify the x,y,z position of the radar in the car.
- **rotation:** Modify the yaw,pitch and roll of the radar.
- **long_range_scan_distance:** the maximum distance the long range radar will detect targets.
- **short_range_scan_distance:** the maximum distance the short range radar will detect targets.
- **long_range_fov:** Field of view (angle) for the long range radar.
- **short_range_fov:** Field of view (angle) for the short range radar.
- **elevation_fov:** Pitch for the field of view (angle).
- **debug_frustum:** Enable to visualize the FOV and distance for long and short range radar. 
- **debug_scan:** Enable to visualize the targets detected while scanning. 
<p>&nbsp;</p>



### Raw Output Data Format

- **Ranges:** A array with the distances to targets.
- **Angle of Arrival:** The angle where the targets are detected.
- **Velocities:** The velocity of the moving and stationary targets.
- **RCS:** Radar cross-section detected by the radar. 
<p>&nbsp;</p>



### Sensor Configuration Examples

#### Paint Target Enable Example
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/configuration/radar/radar_paint_target.jpg" />
</p>
<p>&nbsp;</p>


#### Change on Short and Long Range FOV Example (with debug_frustum enable)
<p class="img_container">
<img class="wide_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/configuration/radar/frustrum_fov.jpg" />
</p>
<p>&nbsp;</p>


#### Change Short and Long Range Distance Example (with debug_frustum enable)
<p class="img_container">
<img class="wide_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/configuration/radar/frustum_distance.jpg" />
</p>
<p>&nbsp;</p>


#### Change Elevation FOV Example (with debug_frustum enable)
<p class="img_container">
<img class="wide_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/configuration/radar/elevation.jpg" />
</p>
<p>&nbsp;</p>


#### debug_scan Enable Example
<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/configuration/radar/debug_scan.jpg" />
</p>