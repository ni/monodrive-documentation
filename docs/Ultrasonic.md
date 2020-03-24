## Ultrasonic Sensor

The location of the sensor can be modified in the "x", "y" and "z" axis with respect to the car.  
The sensor's orientation can be modified in the "yaw", "pitch" and "roll" axis.

### Tree detected by Ultrasonic sensor
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/tree_detected.png" />
</p>

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
- **scan_distance:** The maximum distance the sensor will detect objects [meters]
- **azimuth_fov:** The angle in the x-y plane to detect objects [degrees].
- **elevation_fov:** The angle in the y-z plane to detect objects [degrees]
- **ray_division_y:** The density of the return per cm [ray/cm].
- **ray_division_z:** The density of the return per cm [ray/cm].
- **debug_frustum:** If set to true, the frustum area will be drawn.
- **debug_scan:** If set to true, the scan lines will be drawn.
- **debug_rescan:** If set to true, the scan lines will be drawn only when there is an object enters the **scan_distance**.

### Output Data Format
**Ranges:** Distance in **centimeters** to the closest object in front of the sensor.

### Configuration Examples  

#### Debug Frustum Enabled

<p align="center">
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/debug_frustum.PNG" />
</p>

#### Debug Scan Enabled
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/debug_scan.png" />
</p>

#### Debug Rescan Enabled
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/debug_rescan.PNG" />
</p>

#### Comparison between settings
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/different_settings.PNG" />
</p>


### Ray division Y
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/ray_division_y.png" />
</p>

### Ray division Z
<img src="https://github.com/monoDriveIO/documentation/blob/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/ray_division_z.png" />
</p>

