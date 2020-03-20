## Ultrasonic Sensor

The location of the sensor can be modified in the "x", "y" and "z" axis with respect to the car.  
The sensor's orientation can be modified in the "yaw", "pitch" and "roll" axis.

### Configuration
```
[
 {
  "type": "Ultrasonic",
  "listen_port": 8900,
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
  "fc": 40000.0,
  "pwm_factor": 2.5,
  "range_max": 4.0,
  "range_min": 0.0,
  "max_targets": 93,
  "period": 0.06,
  "uschannel": {
    "minimum_ultrasonic_distance": 0.0,
    "range_scan_distance":4.0,
    "azimuth_fov":30.0,
    "elevation_fov": 0.50,
    "rescan_density": 10.0,
    "max_raycast_hits": 5,
    "debug_scan": true,
    "debug_rescan": true
  }
}
]

```
### Configuration Description
- **fc:**
- **pwm_factor:**
- **range_max:**
- **range_min:**
- **max_targets:**
- **period:**
- **minimum_ultrasonic_distance:**
- **range_scan_distance:**
- **azimuth_fov:**
- **elevation_fov:**
- **rescan_density:**
- **max_raycast_hits:**
- **debug_scan:**
- **debug_rescan:**

### Output Data Format
**Ranges:** Distance to the closest object in front of the sensor.

