Ultrasonic Sensor
-----------------

The location of the sensor can be modified in the "x", "y" and "z" axis
with respect to the car.\
The sensor's orientation can be modified in the "yaw", "pitch" and
"roll" axis.

### Tree detected by Ultrasonic sensor

`<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/tree_detected.png" />`{=html}
```{=html}
</p>
```
### Configuration

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

### Configuration Description

-   **scan\_distance:** The maximum distance the sensor will detect
    objects \[meters\]
-   **azimuth\_fov:** The angle in the x-y plane to detect objects
    \[degrees\].
-   **elevation\_fov:** The angle in the y-z plane to detect objects
    \[degrees\]
-   **ray\_division\_y:** The density of the return per cm \[ray/cm\].
-   **ray\_division\_z:** The density of the return per cm \[ray/cm\].
-   **debug\_frustum:** If set to true, the frustum area will be drawn.
-   **debug\_scan:** If set to true, the scan lines will be drawn.
-   **debug\_rescan:** If set to true, the scan lines will be drawn only
    when there is an object enters the **scan\_distance**.

### Output Data Format

**Ranges:** Distance in **centimeters** to the closest object in front
of the sensor.

### Configuration Examples

#### Debug Frustum Enabled

```{=html}
<p align="center">
```
`<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/debug_frustum.PNG" />`{=html}
```{=html}
</p>
```
#### Debug Scan Enabled

`<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/debug_scan.png" />`{=html}
```{=html}
</p>
```
#### Debug Rescan Enabled

`<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/debug_rescan.PNG" />`{=html}
```{=html}
</p>
```
#### Comparison between settings

`<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/different_settings.PNG" />`{=html}
```{=html}
</p>
```
### Ray division Y

`<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/ray_division_y.png" />`{=html}
```{=html}
</p>
```
### Ray division Z

`<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/configuration/ultrasonic/ray_division_z.png" />`{=html}
```{=html}
</p>
```
State Sensor
------------

The monoDrive state sensor reports information of all objects included
in the **desired\_tags** list. It will ignore the objects in the
**undesired\_tags** list.

#### Configuration

    [
     {
        "type": "State",
        "listen_port": 8700,
        "packet_size": 1472,
        "debug_drawing": true,
        "desired_tags": [
          "vehicle"
        ],
      "undesired_tags": [
      "static"
      ]
    }

    ]

### Raw Output Example

    {
        "frame": [{
            "angular_velocity": [0.009866185486316681, 0.011342001147568226, 0.6717177629470825],
            "brake_input": 0.0,
            "name": "ScenarioVehicle_1",
            "orientation": [0.011697866953909397, -0.02252720296382904, -0.9042208790779114, 0.42631015181541443],
            "position": [15262.076171875, 17526.494140625, 6.538963317871094],
            "tags": ["vehicle", "dynamic", "ego"],
            "throttle_input": 0.0,
            "velocity": [-334.3295593261719, -541.6399536132813, 0.05538097396492958],
            "wheel_input": 0.0,
            "wheel_speed": [15.510863304138184, 23.817401885986328, 25.251663208007813, 19.639755249023438],
            "wheels": [{
                "id": 1,
                "orientation": [-0.026308560743927956, -0.18871694803237915, -0.13943514227867126, -0.97172611951828]
            }, {
                "id": 2,
                "orientation": [0.08424673974514008, 0.6621711254119873, 0.1512315273284912, -0.7290821671485901]
            }]
        }, {
            "angular_velocity": [0.0037936856970191, -0.0072604212909936905, -0.019885115325450897],
            "brake_input": 0.0,
            "name": "ScenarioVehicle2",
            "orientation": [-0.002732698805630207, 0.0007744110189378262, -0.7097418904304504, 0.7044562101364136],
            "position": [15386.1279296875, 9367.8935546875, 10.581443786621094],
            "tags": ["vehicle", "dynamic"],
            "throttle_input": 0.0,
            "velocity": [-12.40577220916748, -1294.0648193359375, 0.004561997950077057],
            "wheel_input": 0.0,
            "wheel_speed": [42.87668228149414, 42.90896987915039, 42.922122955322266, 42.861324310302734],
            "wheels": [{
                "id": 1,
                "orientation": [0.0028412831015884876, 0.659138023853302, 0.0014229556545615196, -0.752015233039856]
            }, {
                "id": 2,
                "orientation": [0.0014198796125128865, -0.9493798017501831, -0.0028250974137336016, -0.31411468982696533]
            }]
        }],
        "game_time": 4.010171890258789,
        "sample_count": 19,
        "time": 1558625081
    }

LiDAR Sensor
------------

The configuration for a LiDAR sensor is modeled after the Velodyne
LiDAR. Currently we only support **16 and 32 laser**.\
The location of the sensor can be modified in the "x", "y" and "z" axis
with respect to the car.\
The sensor's orientation can be modified in the "yaw", "pitch" and
"roll" axis.

Configuration
-------------

    {
      "type": "Lidar",
      "listen_port": 8200,
      "location": {
        "x":-75.0,
        "y":-25.0,
        "z":350.0
      },
      "rotation": {
        "pitch":0.0,
        "yaw":0.0,
        "roll":0.0
      },
      "max_distance": 8000.0,
      "horizontal_resolution": 0.8,
      "rpms": 300.0,
      "n_lasers": 16,
      "reset_angle": 0.0
    }

-   **max\_distance**: The maximum distance, in centimeters, the LiDAR
    laser will travel.
-   **horizontal\_resolution**: The horizontal angle, in degrees, the
    LiDAR must rotate before shooting out the next set of lasers.
    Usually from 0.1 to 0.8.
-   **rpms**: The expected number of revolutions per minute for a full
    360 degree sweep. Controls the expected time between laser lines.
-   **n\_lasers**: The number of lasers the LiDAR sensor shoots out per
    sweep. This can be set to 16 (VLP-16), 32 (HDL-32).
-   **reset\_angle**: The angle that indicates a full revolution
    (i.e. full 360 degree revolution will start at this reported angle).

### Raw Output Data Format

Each data packet from the LiDAR sensor contains **1206 bytes**.\
There are 12 data blocks, each block is **100 bytes** and there are **4
bytes** for the timestamp and **2 bytes** called factory bytes. The
number of packets for a revolution depends on the **horizontal
revolution**.

-   [VLP-16 Manual Download](http://velodynelidar.com/vlp-16.html)
-   [HDL-32E Manual Download](http://velodynelidar.com/hdl-32e.html)

Visualize LiDAR output
----------------------

1.  Download [VeloView](https://www.paraview.org/VeloView/).
2.  Run VeloView
3.  Click `Sensor Stream`, and select the correct configuration (Puck
    Hi-Res or HDL-32).
4.  Run the simulator and client normally when the stream data starts
    coming through VeloView will populate with the LiDAR data.

```{=html}
<p align="center">
```
`<img src="https://github.com/monoDriveIO/Client/raw/master/WikiPhotos/lidarsensor.PNG"  height="400" />`{=html}
```{=html}
</p>
```
