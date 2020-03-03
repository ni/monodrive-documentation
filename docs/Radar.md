## Radar Sensor

The configuration for a radar sensor.

```
[
    {
  "type": "Radar",
  "gpu_number":0,
  "listen_port": 8301,
  "send_radar_cube": false,
  "paint_targets": false,
  "target_paint_lifetime": 0.5,
  "location": {
    "x": 250.0,
    "y": 0.0,
    "z": 50.0
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
  "max_targets": 250,
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
    "long_range_scan_distance": 150.0,
    "short_range_scan_distance": 60.0,
    "long_range_fov": 20.0,
    "short_range_fov": 60.0,
    "elevation_fov": 10.0,
    "rescan_density": 2.0,
    "debug_frustum": false,
    "debug_scan": false,
    "debug_rescan": false
  }
}
]
```
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

### Sensor configuration examples

#### Paint target enable
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/sensor_update/WikiPhotos/LV_client/sensors/configuration/radar/radar_paint_target.jpg" />
</p>

#### Change on short and long range FOV with debug_frustum enable 
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/sensor_update/WikiPhotos/LV_client/sensors/configuration/radar/frustrum_fov.jpg" />
</p>

#### Change on short and long range distance with debug_frustum enable 
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/sensor_update/WikiPhotos/LV_client/sensors/configuration/radar/frustum_distance.jpg" />
</p>

#### Change elavation fov with debug_frustum enable 
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/sensor_update/WikiPhotos/LV_client/sensors/configuration/radar/elevation.jpg" />
</p>

#### debug_scan enable 
<p align="center">
<img src="https://github.com/monoDriveIO/client/raw/sensor_update/WikiPhotos/LV_client/sensors/configuration/radar/debug_scan.jpg" />
</p>


### Raw Output Data Format

- **0...NumberOfSweeps as i:**
  - **Bytes 8i-4+8i:** Real bits represented as a float in single precision IEEE-754 format.
  - **Bytes 4+8i-8+8i:** Imaginary bits represented as a float in single precision IEEE-754 format.
