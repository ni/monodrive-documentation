
### Parameter Format

Every sensor follows the following format as the base for what you can
do with a sensor. The individual sensor pages will explain their own
parameters in addition to these base parameters.

    {
          "type": string,
          "id": string,
          "packet_size": int,
          "listen_port": int,
          "display_process": bool,
          "sensor_process": bool,
          "location": {
            "x": float,
            "y": float,
            "z": float
          },
          "rotation": {
            "pitch": float,
            "yaw": float,
            "roll": float
          },
          "fps": float
    }

-   **type**: The values of type can be: `MultiCamera`, `Camera`,
    `Semantic`, `Lidar`, `IMU`, `GPS`, `RPM`, `Radar`, `Waypoint`, or
    `BoundingBox` depending on what type of sensor you are trying to set
    up.
-   **listen\_port**: The TCP or UDP port that the simulator must send
    the data through for that sensor to get the data to the client.
-   **location**: The relative location for which the sensor will be
    placed on the vehicle.
    -   *x*: The x position.
    -   *y*: The y position.
    -   *z*: The z position.
-   **rotation**: The relative rotation the sensor will have on the
    vehicle.
    -   *pitch*: The pitch.
    -   *yaw*: The yaw.
    -   *roll*: The roll.

Camera Sensor
-------------

The monoDrive camera sensor supports 4 types of image output: - RGB -
Semantic segmantation - Grayscale - Depth camera

The image output support different sizes i.e. 512x512 pixels.\
The location of the sensor can be modified in the "x", "y" and "z" axis
with respect to the car.\
The sensor's orientation can be modified in the "yaw", "pitch" and
"roll" axis.

#### RGB camera

Provides a RGBA camera stream with optional bounding boxes for dynamic
objects in the scene.

    [
       {
        "type": "Camera",
        "listen_port": 8000,
        "location": {
          "x": 0.0,
          "y": 0.0,
          "z": 250.0
        },
        "rotation": {
          "pitch": 0.0,
          "yaw": 0.0,
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
        "channels": "rgba"
      }
    ]

```{=html}
<p align="center">
```
`<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/camerasensor.PNG"  height="400" />`{=html}
```{=html}
</p>
```
#### Semantic camera

Provides a grayscale camera stream.

    [
      {
         "type": "SemanticCamera",
         "listen_port": 8051,
         "location": {
           "x": -800.0,
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
        "max_distance": 50000.0,
        "dynamic_range":  50,
        "fov": 60.0,
        "focal_length": 9.0,
        "fstop": 1.4,
        "min_shutter":  0.000500,
        "max_shutter":  0.001400,
        "sensor_size":  9.07,
       "channels" : "rgba"
     }

    ]

```{=html}
<p align="center">
```
`<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/semanticcamerasensor.PNG"  height="400" />`{=html}
```{=html}
</p>
```
#### Grayscale

Provides a grayscale camera stream.

    [
      {
         "type": "Camera",
         "listen_port": 8120,
         "location": {
           "x": -800.0,
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
        "max_distance": 50000.0,
        "dynamic_range":  50,
        "fov": 60.0,
        "focal_length": 9.0,
        "fstop": 1.4,
        "min_shutter":  0.000500,
        "max_shutter":  0.001400,
        "sensor_size":  9.07,
       "channels" : "gray"

     }

    ]

```{=html}
<p align="center">
```
`<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/camera_grayscale.png"  height="400" />`{=html}
```{=html}
</p>
```
### Depth Camera

Provides a grayscale camera stream with different intensity depending on
the distance to the objects.

    [
      {
         "type": "DepthCamera",
          "listen_port": 8120,
          "location": {
           "x": -800.0,
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
        "fov": 60.0
     }

    ]

```{=html}
<p align="center">
```
`<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/camera_depth.png"  height="400" />`{=html}
```{=html}
</p>
```
Annotation
----------

All cameras support the annotation feature. The annotation feature will
give you bounding boxes for all the dynamic objects in the scene. Add
the following json tags to any camera configuration.

    "annotation": {
        "desired_tags": [
        "traffic_sign", "vehicle"
        ],
        "far_plane": 10000.0,
        "include_tags": true
    }

Configuration Tags
------------------

-   **stream\_dimensions**: The dimensions of which the stream will
    stream over TCP. These dimensions are set in a dictionary with keys
    x and y with float values.
-   **dynamic\_range**: The dynamic range of the camera in decibels.
-   **fov**: (optional) If set this will override the angle of view
    calculated from the sensor size and lens focal length.
-   **focal\_length**: The focal length of the lens in millimeters.
-   **fstop**: The f-stop value for the lens in stops.
-   **min\_shutter**: The minimum shutter speed, in seconds, for dynamic
    exposure.
-   **max\_shutter**: The maximum shutter speed, in seconds, for dynamic
    exposure.
-   **sensor\_size**: The size of the camera sensor in millimeters.
-   **channels**: The output color of the camera, needs to be "rgba" or
    "grayscale".

### annotation

If set this would enable the anottation of objects in the scene.

-   **desired\_tags**: Tags of objects to be classified. Refer to the
    table below to obtain supported tags.

  Object          Tag Name
  --------------- ---------------------
  car             car, vehicle
  motorcycle      motorcycle, vehicle
  bus             bus, vehicle
  person          person, vru
  bicycle         bicycle, vru
  traffic signs   traffic\_sign, tcd

-   **far\_plane**: The maximum distance will be consider for annotation
    on the X axis of the camera.
-   **include\_tags**: Flag to include the tags of the objects
    classified.
-   **include\_obb**: Flag to include a 3D object-oriented bounding box
    in the output.
-   **cull\_partial\_frame**: If set to true an object only partially
    visible in the camera frame will be toss out that annotation.

### Raw Output Data Format

-   **time\_stamp (int):** Timestamp representing milliseconds since
    Sunday.
-   **game\_time (float):** Current game time of simulator, this value
    will be more prominent.
-   **Annotation (string):** Return information of the classified
    objects from the scene, the 2D bounding box, the 3D oriented boundig
    box, the tag name, etc.
-   **image (List\<List\<float`<float>`{=html}\>\>):** A 3D list that
    represents a single image following the format of
    **stream\_dimensions\_x** x **stream\_dimensions\_y** x 4

### Camera configuration examples.

#### FOV

```{=html}
<p align="center">
```
`<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/configuration/camera/FOV.png" />`{=html}
```{=html}
</p>
```
#### far\_plane

```{=html}
<p align="center">
```
`<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/configuration/camera/far_plane.png" />`{=html}
```{=html}
</p>
```
#### cull\_partial\_frame

```{=html}
<p align="center">
```
`<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/configuration/camera/cull_partial_frame.png" />`{=html}
```{=html}
</p>
```
Radar Sensor
------------

The configuration for a radar sensor.

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

-   **gpu\_number:** Select the specific GPU number(0-N) where each
    radar sensor will run.
-   **send\_radar\_cube:** Enable to obtain the radar cube from the
    simulator.
-   **paint\_targets:** Tool to draw a box around the targets detected.
-   **target\_paint\_lifetime:** Specify how long a painted target box
    will be rendered for in the engine, in seconds.
-   **location:** Modify the x,y,z position of the radar in the car.
-   **rotation:** Modify the yaw,pitch and roll of the radar.
-   **long\_range\_scan\_distance:** the maximum distance the long range
    radar will detect targets.
-   **short\_range\_scan\_distance:** the maximum distance the short
    range radar will detect targets.
-   **long\_range\_fov:** Field of view (angle) for the long range
    radar.
-   **short\_range\_fov:** Field of view (angle) for the short range
    radar.
-   **elevation\_fov:** Pitch for the field of view (angle).
-   **debug\_frustum:** Enable to visualize the FOV and distance for
    long and short range radar.
-   **debug\_scan:** Enable to visualize the targets detected while
    scanning.

### Sensor configuration examples

#### Paint target enable

```{=html}
<p align="center">
```
`<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/configuration/radar/radar_paint_target.jpg" />`{=html}
```{=html}
</p>
```
#### Change on short and long range FOV with debug\_frustum enable

```{=html}
<p align="center">
```
`<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/configuration/radar/frustrum_fov.jpg" />`{=html}
```{=html}
</p>
```
#### Change on short and long range distance with debug\_frustum enable

```{=html}
<p align="center">
```
`<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/configuration/radar/frustum_distance.jpg" />`{=html}
```{=html}
</p>
```
#### Change elavation fov with debug\_frustum enable

```{=html}
<p align="center">
```
`<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/configuration/radar/elevation.jpg" />`{=html}
```{=html}
</p>
```
#### debug\_scan enable

```{=html}
<p align="center">
```
`<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/configuration/radar/debug_scan.jpg" />`{=html}
```{=html}
</p>
```
### Output Data Format

-   **Ranges:** A array with the distances to targets.
-   **Angle of Arrival:** The angle where the targes are detected.
-   **Velocities:** The velocity of the moving and stationary targets.
-   **RCS:** Radar cross-section detected by the radar.

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
Viewport Sensor
---------------

The monoDrive Viewport sensor allows you to configure the information
displayed on your screen.

### Configuration

    [
    {
      "type": "ViewportCamera",
      "listen_port": 0,
     "location": {
          "x": 0.0,
          "y": 0.0,
          "z": 250.0
        },
        "rotation": {
          "pitch": 0.0,
          "yaw": 0.0,
          "roll": 0.0
        },
        "stream_dimensions": {
          "x": 512.0,
          "y": 512.0
        },
        "dynamic_range": 50,
        "fov": 60.0,
        "focal_length": 9.0,
        "fstop": 1.4,
        "min_shutter": 0.000500,
        "max_shutter": 0.001400,
        "sensor_size": 9.07,
        "channels": "rgba"
    }

    ]

Ouput Examples
--------------

Image on the left is the camera output and on the right the image on the
Viewport on the VehicleAI Editor.

```{=html}
<p align="center">
```
`<img src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/configuration/viewport/comparison_camera_viewport.png" />`{=html}
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

GPS Sensor
----------

The GPS sensor provides the location information from the EGO vehicle
such as latitude and longitude.\
The location of the sensor can be modified in the "x", "y" and "z" axis
with respect to the car.\
The sensor's orientation can be modified in the "yaw", "pitch" and
"roll" axis.

Configuration for a GPS sensor.
-------------------------------

    [
       {
        "type": "GPS",
        "listen_port": 8400,
        "location": {
          "x": -75.0,
          "y": -25.0,
          "z": 245.0
        },
        "rotation": {
          "pitch": 0.0,
          "yaw": 0.0,
          "roll": 0.0
        }
      }
    ]

### Raw Output Data Format

The sensor output has 78 bytes, where 12 bytes correspond to the header
of the message and 66 correspond to data from the sensor.\
Following the data format found
[here](https://github.com/swift-nav/libsbp/raw/master/docs/sbp.pdf).

  -----------------------------------------------------------------------
  Type                                Name
  ----------------------------------- -----------------------------------
  Byte 0-1                            preamble

  Bytes 1-3                           MSG\_POS\_LLH

  Byte 5-6                            Hash of the sensor id

  Byte 5-6                            Length of the payload

  Bytes 6-14                          Latitude bits represented as a
                                      double in double precision IEEE-754
                                      format.

  Bytes 14-22                         Longitude bits represented as a
                                      double in double precision IEEE-754
                                      format

  Bytes 22-30                         Elevation bits represented as a
                                      double in double precision IEEE-754
                                      format

  Bytes 30-38                         WorldLocation\_x

  Bytes 38-46                         WorldLocation\_y

  Bytes 36-40                         forward\_x

  Bytes 40-44                         forward\_y

  Bytes 44-48                         forward\_z

  Bytes 48-52                         ego\_yaw

  Bytes 52-56                         ego\_speed

  Bytes 56-58                         horizontal\_acceleration

  Bytes 58-60                         vertical\_acceleration

  Bytes 60-62                         Number of satellites used for
                                      signal

  Bytes 62-64                         Fixed more status

  Bytes 64-66                         CRC
  -----------------------------------------------------------------------

IMU Sensor
----------

The monoDrive reports the Ego's acceleration in x,y and z axis. As well
as the angular velocity in the same axis.\
The location and orientation of the sensor can be modified in the "x",
"y" and "z" axis with respect to the car.

IMU configuration
-----------------

The configuration for an IMU sensor.

    [
      {
        "type": "IMU",
        "listen_port": 8500,
        "location": {
          "x":-75.0,
          "y":-25.0,
          "z":245.0
        },
        "rotation": {
          "pitch":0.0,
          "yaw":0.0,
          "roll":0.0
        }
      }

    ]

### Raw Output Data Format

Following the data format found
[here](http://files.microstrain.com/dcp/Inertia-Link-3DM-GX2-data-communications-protocol.pdf).

  -----------------------------------------------------------------------
  Byte                                Description
  ----------------------------------- -----------------------------------
  Byte 0-1                            Start Byte

  Bytes 1-5                           x acceleration bits represented as
                                      a float in single precision
                                      IEEE-754 format.

  Bytes 5-9                           y acceleration bits represented as
                                      a float in single precision
                                      IEEE-754 format.

  Byte 9-13                           z acceleration bits represented as
                                      a float in single precision
                                      IEEE-754 format.

  Bytes 13-17                         x angular velocity bits represented
                                      as a float in single precision
                                      IEEE-754 format.

  Bytes 17-21                         y angular velocity bits represented
                                      as a float in single precision
                                      IEEE-754 format.

  Bytes 21-25                         z angular velocity bits represented
                                      as a float in single precision
                                      IEEE-754 format.

  Bytes 25-29                         Timestamp as indicated in the above
                                      link.

  Bytes 29-31                         Checksum.

  Byte 31-35                          Time of week.
  -----------------------------------------------------------------------

RPM Sensor
----------

The monoDrive sensor give you information on the Revolutions per second
on the spexified wheel. You can obtain information of the four wheels on
the car (0-3).

Configuration
-------------

    [
    {
      "type": "RPM",
      "listen_port": 8600,
      "location": {
        "x": 0,
        "y": 0,
        "z": 0
      },
      "rotation": {
        "pitch": 0,
        "yaw": 0,
        "roll": 0
      },
      "wheelNumber": 3
    }
    ]

### Raw Output Data Format

  Byte        Description
  ----------- -----------------------------------------------------
  Bytes 0-4   The wheel number for the RPM sensor.
  Bytes 4-8   The wheel speed in RPM for the specific RPM sensor.

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
