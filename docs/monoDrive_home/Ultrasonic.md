# Ultrasonic

Provides range to closest object from a single ultrasonic sensor.

## Configuration 

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

- **fc:** The sampling frequency of the sensor in hertz.
- **pwm_factor:** The coefficient to the sampling frequency for pulse width modulation.
- **max_ultrasonic_returns:** The maximum number of returns from the shooting bouncing ray cast. Lower numbers will increase performance.
- **sbr:** JSON object specifying shooting bouncing ray parameters for the sensor.
    - **scan_distance:** The maximum distance the sensor will detect objects in meters.
    - **azimuth_fov:** The angle in the x-y plane to detect objects in degrees.
    - **elevation_fov:** The angle in the y-z plane to detect objects in degrees.
    - **ray_division_y:** The density of the return per centimeter in rays per centimeter.
    - **ray_division_z:** The density of the return per centimeter in rays per centimeter.
    - **debug_frustum:** If set to true, the frustum area will be drawn.
    - **debug_scan:** If set to true, the scan lines will be drawn.
    - **debug_rescan:** If set to true, the scan lines will be drawn only when there is an object enters the **scan_distance**.

## Raw Output
```
{
  "targets":[
      {"range":300.0}
    ]
} 
```

**ranges:** Array containing distance in centimeters to the closest object in front of the sensor. A value of -1 indicates no return from the sensor.

### Configuration Examples  

#### Tree Detected by Ultrasonic Sensor
<p class="img_container">
<img class="wide_img" src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/tree_detected.png" />
</p>

#### Debug Frustum Enabled
<p class="img_container">
<img class="wide_img" src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/debug_frustum.PNG" />
</p>

#### Debug Scan Enabled
<p class="img_container">
<img class="wide_img" src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/debug_scan.png" />
</p>

#### Debug Rescan Enabled
<p class="img_container">
<img class="wide_img" src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/debug_rescan.PNG" />
</p>

#### Comparison Between Settings
<p class="img_container">
<img class="wide_img" src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/different_settings.PNG" />
</p>


### Ray Division Y
<p class="img_container">
<img class="wide_img" src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/ray_division_y.png" />
</p>

### Ray Division Z
<p class="img_container">
<img class="wide_img" src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/ray_division_z.png" />
</p>
