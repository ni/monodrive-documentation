## Ultrasonic Sensor

The location of the sensor can be modified in the "x", "y" and "z" axis with respect to the car.  
The sensor's orientation can be modified in the "yaw", "pitch" and "roll" axis.

### Configuration
```
[
   {
  "type": "Ultrasonic",
  "listen_port": 8977,
  "location": {
    "x": 130.0,
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
  "max_ultrasonic_returns": 93,
  "period": 0.06,
   "sbr": {
   "scan_distance":4.0,
   "azimuth_fov":30.0,
   "elevation_fov": 0.50,
    "ray_division_y": 5.0,
    "ray_division_z": 5.0,
    "debug_frustum": false,
    "debug_scan": false,
    "debug_rescan": false
  }
}
```
### Configuration Description
- **fc:**
- **pwm_factor:**
- **max_ultrasonic_returns**
- **period:**
- **scan_distance:**
- **azimuth_fov:**
- **elevation_fov:**
- **ray_division_y:**
- **ray_division_z:**
- **debug_frustum:**
- **debug_scan:**
- **debug_rescan:**


### Output Data Format
**Ranges:** Distance to the closest object in front of the sensor.

