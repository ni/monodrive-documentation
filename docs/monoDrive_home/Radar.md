# Radar

Provides Radar data from an electronically scanning Radar. This sensor is capable 
of producing a list of detected targets and the raw Radar data cube that was 
processed to produce the list.

## Configuration

```json
{
    "bandwidth": 250000000,
    "description": "",
    "elements": 8,
    "fc": 77000000000.0,
    "fs": 50000000.0,
    "gpu_number": 0,
    "listen_port": 0,
    "location": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "max_radar_returns": 500,
    "nearest_target_label_radius": 50.0,
    "num_samples_per_sweep": 345,
    "num_sweeps": 32,
    "paint_targets": false,
    "receiver": {
        "aperture": 0.000798,
        "gain": 10.0,
        "nb": 74000000.0,
        "nf": 10.0,
        "noise_temp": 290.0
    },
    "ros": {
        "advertise": true,
        "message_type": "",
        "publish_to_ros": false,
        "queue_size": 1,
        "send_tf": false,
        "topic": ""
    },
    "rotation": {
        "pitch": 0.0,
        "roll": 0.0,
        "yaw": 0.0
    },
    "sbr": {
        "debug_frustum": false,
        "debug_rescan": false,
        "debug_scan": false,
        "elevation_fov": 10.0,
        "long_range_fov": 30.0,
        "long_range_scan_distance": 60.0,
        "ray_division_y": 10.0,
        "ray_division_z": 10.0,
        "short_range_fov": 60.0,
        "short_range_scan_distance": 30.0
    },
    "send_radar_cube": false,
    "sweep_time": 6.9e-06,
    "target_paint_lifetime": 0.5,
    "transmitter": {
        "aperture": 0.000859,
        "gain": 13.5,
        "peak_power": 5.0
    },
    "type": "Radar",
    "wait_for_fresh_frame": true
}
```

- **gpu_number:** Specifies the specific GPU number(0-N) that will process the Radar data.
- **send_radar_cube:** If true, the the radar data cube will be sent in addition to the Radar target list.
- **paint_targets:** If true, target boxes will be drawn on Radar targets in the viewport and camera sensors.
- **target_paint_lifetime:** The total number of seconds to paint a Radar target when detected.
- **nearest_target_paint_radius:** The total size in centimeters of the box to draw around the center of a Radar target.
- **num_samples_per_sweep:** The total number of shooting bouncing rays to trace for a single Radar sweep.
- **fs:** The sampling frequency of the Radar.
- **fc:** The center frequency of the Radar.
- **num_sweeps:** The total number of scanning sweeps to make per update.
- **sweep_time:** The total amount of time in seconds each sweep will take.
- **bandwidth:** The total bandwidth in Hz for a single sweep.
- **max_radar_returns:** The total number of shooting bouncing rays to trace for an entire Radar return.
- **elements:** The total number of receiver elements for the Radar.
- **transmitter:** JSON value for Radar transmitter properties
    - **aperture:** The total aperture radius in centimeters for Radar transmit array.
    - **gain:** The total gain over the transmission signal in dB.
- **receiver:** JSON value for Radar transmitter properties
    - **aperture:** The total aperture radius in centimeters for Radar receive array.
    - **noise_temp:** Coefficient to the exponent of the noise floor. Controls the level of the noise floor in the data.
    - **gain:** The total gain applied to the receivers in dB.
- **sbr:** JSON value for shooting bouncing ray properties of the Radar ray traces
    - **long_range_scan_distance:** The maximum distance the long range Radar will detect targets in meters.
    - **short_range_scan_distance:** The maximum distance the short range Radar will detect targets in meters.
    - **long_range_fov:** Field-of-view in degrees of the long range Radar.
    - **short_range_fov:** Field-of-view in degrees of the short range Radar.
    - **elevation_fov:** Pitch of the field-of-view for long and short range Radar, in degrees.
    - **ray_division_y:** The span in degrees of rays to cast in the y-dimension (horizontal) 
    - **ray_division_z:** The span in degrees of rays to cast in the y-dimension (vertical).
    - **noise_ray_division_y:** The span in degrees of noise to add to rays in the y-dimension
    - **noise_ray_division_z:** The span in degrees of noise to add to rays in the z-dimension
    - **debug_frustum:** If true, a visualization of the field-of-view of the Radar will be drawn on the viewport and camera sensors.
    - **debug_scan:** If true, a visualization of the rays being cast by the SBR will be drawn on the viewport and camera sensors.
    - **debug_rescan:** If true, a visualization of the rescan rays being cast by the SBR will be drawn on the viewport and camera sensors.


## Raw Output

By default, the Radar will return JSON containing the Radar returns. If 
`send_radar_cube` is set, the Radar data cube will also be sent.

### Target List

```
{
   "gt_targets":[
      {
         "aoa":8.1068286895752,
         "range":83.2458343505859,
         "rcs":0.0,
         "target_ids":[
            "subcompact_monoDrive_01_C_0"
         ],
         "velocity":-3.06528854370117
      },
      {
         "aoa":0.472435683012009,
         "range":94.7941360473633,
         "rcs":0.0,
         "target_ids":[
            "prp_streetLight96"
         ],
         "velocity":9.99999974737875e-05
      }
   ],
   "target_list":[
      {
         "aoa":-6.1800012588501,
         "range":80.4576187133789,
         "rcs":5.92999982833862,
         "target_ids":[
            "sedan_monoDrive_01_B"
         ],
         "velocity":5.46999979019165
      },
      {
         "aoa":-12.6000003814697,
         "range":41.2400016784668,
         "rcs":0.639999985694885,
         "target_ids":[
            "M_ShippingContainer72"
         ],
         "velocity":-0.0
      }
   ]
}
```

The JSON portion of the Radar return contains ground truth and actual detected 
Radar returns. All arrays in the returns are indexed by target number (i.e. 
index 0 of `aoas` corresponds with index 0 of `ranges` and so on.)

- **game_time:** The time in seconds since the simulation started when this sample was acquired
- **message:** JSON containing the Radar return results
    - **gt_target_list:** JSON containing ground truth targets within the Radar range
        - **aoas:** Angle of arrival of each target in degrees
        - **ranges:** The range to each target in centimeters
        - **rcs:** The radial cross-section of the Radar return to each target
        - **target_ids:** The string representing the name of the actor that generated the target
        - **velocities:** The velocity in meters per second of each target
    - **target_list:** JSON containing the targets detected in the processed Radar signal
        - **aoas:** Angle of arrival of each target in degrees
        - **ranges:** The range to each target in centimeters
        - **rcs:** The radial cross-section of the Radar return to each target
        - **target_ids:** Array of target information for each target
            - **target_id:** The string representing the name of the actor that generated the target
        - **velocities:** The velocity in meters per second of each target
  **time:** The time in UTC seconds when this sample was acquired

### Radar Data Cube

The Radar data cube contains the complex values for each element in the 
[3D Radar cube](https://www.mathworks.com/help/phased/gs/radar-data-cube.html).

The data contains a binary array that is 

```bash
    Number of Sweeps  *  Number of Samples per Sweep  *  Number of Elements
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