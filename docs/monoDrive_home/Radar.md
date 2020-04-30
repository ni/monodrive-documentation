# RADAR

Provides RADAR data from electronically scanning RADAR. This sensor is capable 
of producing a list of detected targets and the raw RADAR data cube that was 
processed to produce the list.

## Configuration

```
[
  {
    "type": "Radar",
    "listen_port": 8302,
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
    "gpu_number":0,
    "send_radar_cube": false,
    "paint_targets": false,
    "target_paint_lifetime": 0.5,
    "nearest_target_paint_radius": 50.0,
    "num_samples_per_sweep":345,
    "fs": 50000000,
    "fc": 77000000000.0,
    "num_sweeps": 32,
    "sweep_time" : 0.0000069,
    "bandwidth" : 250000000,
    "max_radar_returns": 500,
    "elements": 8,
    "transmitter": {
      "aperture": 0.000859,
      "gain": 13.50
    },
    "receiver": {
      "aperture": 0.000798,
      "noise_temp": 290.0,
      "gain": 10.0
    },
    "sbr": {
      "long_range_scan_distance": 60.0,
      "short_range_scan_distance": 30.0,
      "long_range_fov": 60.0,
      "short_range_fov": 30.0,
      "elevation_fov": 10.0,
      "ray_division_y": 10.0,
      "ray_division_z": 10.0,
      "noise_ray_division_y": 200.0,
      "noise_ray_division_z": 200.0,
      "debug_frustum": false,
      "debug_scan": false,
      "debug_rescan": false
    }
  }
]
```

- **gpu_number:** Specifies the specific GPU number(0-N) that will process the RADAR data.
- **send_radar_cube:** If true, then the radar data cube will be sent in addition to the RADAR target list.
- **paint_targets:** If true, target boxes will be drawn on RADAR targets in the viewport and camera sensors.
- **target_paint_lifetime:** The total number of seconds to paint a RADAR target when detected.
- **nearest_target_paint_radius:** The total size, in centimeters of the box to draw around the center of a RADAR target.
- **num_samples_per_sweep:** The total number of shooting bouncing rays to trace for a single RADAR sweep.
- **fs:** The sampling frequency of the RADAR.
- **fc:** The center frequency of the RADAR.
- **num_sweeps:** The total number of scanning sweeps to make per update.
- **sweep_time:** The total amount of time, in seconds, each sweep will take.
- **bandwidth:** The total bandwidth, in Hz, for a single sweep.
- **max_radar_returns:** The total number of shooting bouncing rays to trace for an entire RADAR return.
- **elements:** The total number of receiver elements for the RADAR.
- **transmitter:** JSON value for RADAR transmitter properties
    - **aperture:** The total aperture radius, in centimeters, for RADAR transmit array.
    - **gain:** The total gain over the transmission signal in dB.
- **receiver:** JSON value for RADAR transmitter properties
    - **aperture:** The total aperture radius, in centimeters, for RADAR receive array.
    - **noise_temp:** coefficient to the exponent of the noise floor. Controls the level of the noise floor in the data.
    - **gain:** The total gain applied to the receivers, in dB.
- **sbr:** JSON value for shooting bouncing ray properties of the RADAR ray traces
    - **long_range_scan_distance:** The maximum distance the long range RADAR will detect targets, in meters.
    - **short_range_scan_distance:** The maximum distance the short range RADAR will detect targets, in meters.
    - **long_range_fov:** Field-of-view, in degrees, of the long range RADAR.
    - **short_range_fov:** Field-of-view, in degrees, of the short range RADAR.
    - **elevation_fov:** Pitch of the field-of-view for long and short range RADAR, in degrees.
    - **ray_division_y:** The span, in degrees, of rays to cast in the y-dimension (horizontal) 
    - **ray_division_z:** The span, in degrees, of rays to cast in the y-dimension (vertical).
    - **noise_ray_division_y:** The span, in degrees, of noise to add to rays in the y-dimension
    - **noise_ray_division_z:** The span, in degrees, of noise to add to rays in the z-dimension
    - **debug_frustum:** If true, a visualization of the field-of-view of the RADAR will be drawn on the viewport and camera sensors.
    - **debug_scan:** If true, a visualization of the rays being cast by the SBR will be drawn on the viewport and camera sensors.
    - **debug_rescan:** If true, a visualization of the rescan rays being cast by the SBR will be drawn on the viewport and camera sensors.


## Raw Output

By default, the RADAR will return JSON containing the RADAR returns. If 
`send_radar_cube` is set, the RADAR data cube will also be sent.

### Target List

```
{
    "game_time": 3842.4619140625,
    "message": {
        "gt_target_list": {
            "aoas": [
                -12.34434986114502,
                -0.6279215812683105,
                12.690101623535156,
                12.690101623535156,
                19.021120071411133
            ],
            "ranges": [
                50.1065673828125,
                115.46038055419922,
                52.34439468383789,
                52.34439468383789,
                52.95979309082031
            ],
            "rcs": [
                0,
                0,
                0,
                0,
                0
            ],
            "target_ids": [
                "M_ShippingContainer46",
                "SMM_RoadMarkingMeshes_5",
                "Stop_36_Sign11",
                "Stop_36_Sign11",
                "M_ShippingContainer47"
            ],
            "velocities": [
                -0.002745997626334429,
                -0.002137478906661272,
                0.007698351517319679,
                0.007698351517319679,
                0.010167188011109829
            ]
        },
        "target_list": {
            "aoas": [
                -12.1899995803833,
                15.029999732971191
            ],
            "ranges": [
                48.91999816894531,
                52.959999084472656
            ],
            "rcs": [
                12.670000076293945,
                1.8299999237060547
            ],
            "target_ids": [
                {
                    "target_id": [
                        "Stop_36_Sign11",
                    ]
                },
                {
                    "target_id": [
                        "M_ShippingContainer47"
                    ]
                }
            ],
            "velocities": [
                -0.0,
                0.0
            ]
        }
    },
    "time": 1588268867
}
```

The JSON portion of the RADAR return contains ground truth and actual detected 
RADAR returns. All arrays in the returns are indexed by target number (i.e. 
index 0 of `aoas` corresponds with index 0 of `ranges` and so on.)

- **game_time:** The time, in seconds since the simulation started, the sample was acquired
- **message:** JSON containing the RADAR return results
    - **gt_target_list:** JSON containing ground truth targets within the RADAR range
        - **aoas:** Angle of arrival of each target, in degrees
        - **ranges:** The range to each target in centimeters
        - **rcs:** The radial cross-section of the RADAR return to each target
        - **target_ids:** The string representing the name of the actor that generated the target
        - **velocities:** The velocity, in meters per second, of each target
    - **target_list:** JSON containing the targets detected in the processed RADAR signal
        - **aoas:** Angle of arrival of each target, in degrees
        - **ranges:** The range to each target in centimeters
        - **rcs:** The radial cross-section of the RADAR return to each target
        - **target_ids:** Array of target information for each target
            - **target_id:** The string representing the name of the actor that generated the target
        - **velocities:** The velocity, in meters per second, of each target
  **time:** The time, in UTC seconds, the sample was acquired

### RADAR Data Cube

The RADAR data cube contains the complex values for each element in the 
[3D RADAR cube](https://www.mathworks.com/help/phased/gs/radar-data-cube.html).

The data contains a binary array that is 

```bash
    Number of Sweeps  x  Number of Samples per Sweep  x  Number of Elements
```

in size. Each value in the array is two 32-bit floating point numbers 
representing the real and imaginary parts of each return value.

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