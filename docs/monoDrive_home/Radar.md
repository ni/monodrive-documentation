# Radar

Provides Radar data from an electronically scanning Radar. This sensor is capable 
of producing a list of detected targets and the raw Radar data cube that was 
processed to produce the list.

## Configuration

```json
{
    "type": "Radar",
    "bandwidth": 250000000,
    "elements": 8,
    "fc": 77000000000.0,
    "fs": 50000000.0,
    "gpu_number": 0,
    "listen_port": 8302,
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
    "run_tracker": true,
    "tracker": {
        "filter_size": 8,
        "kalman_alpha": 10.0,
        "kinematic_iir_alpha": 0.05,
        "min_cluster_distance": 25.0,
        "min_detections_per_cluster": 2,
        "missed_detections_threshold": 5,
        "vel_coeff": 0.7,
        "x_dir_coeff": 10.0,
        "y_dir_coeff": 4.0,
        "write_debug_json": "",
    },
    "rois": [
        {
            "max_angle": 10.0,
            "min_angle": 0.0,
            "max_range": 20.0,
            "min_range": 1.0,
            "roi_name": "test_left_roi"
        },
        {
            "max_angle": 0.0,
            "min_angle": -10.0,
            "max_range": 20.0,
            "min_range": 1.0,
            "roi_name": "test_right_roi"
        }
    ]
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
- **sbr:** JSON value for shooting bouncing ray properties of the Radar ray traces.
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
- **run_tracker:** If true the radar tracker will run during simulation and tracks will be output.
- **tracker**: Tracker settings for the radar
    - **filter_size**: The total number of samples to buffer for tracking.
    - **kalman_alpha**: The integration coefficient to the extended Kalman filter (EKF) for tracking.
    - **kinematic_iir_alpha**: The integration coefficient to the kinematic IIR filter for track properties.
    - **min_cluster_distance**: The minimum distance between separate clusters of returns. Represents the `x distance * y distance * velocity` for clustering.
    - **min_detections_per_cluster**: The minimum number of detections needed to form a cluster.
    - **missed_detections_threshold**: The total number of missed detections before a track is dropped.
    - **vel_coeff**: The coefficient for the velocity thresholding in the EKF.
    - **x_dir_coeff**: The coefficient for the x distance thresholding in the EKF.
    - **y_dir_coeff**: The coefficient for the y distance thresholding in the EKF.
    - **write_debug_json**: If set to a valid filename, debug output from the tracker will be written to disk when the simulation is stopped.
- **rois**: Regions of interest for detections. If there are valid detections within the defined regions then the ROI will be present in the tracker output. The ROI is defined as positive clockwise from -180 to 180 degrees starting behind the vehicle.
    - **max_angle**: The maximum angle in degrees to bound the region.
    - **min_angle**: The minimum angle in degrees to bound the region.
    - **max_range**: The maximum range in meters to bound the region.
    - **min_range**: The minimum range in meters to bound the region.
    - **roi_name**: The unique ID for this region to populate in the output.

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
      },
      ...
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
      },
      ...
   ],
   "tracks": [
       {
           "heading": 0.0,
           "lifetime_seconds": 0.0,
           "track_id": 1,
           "velocity": 0.0,
           "x_range": -0.39406856894493103,
           "y_range": 14.31589412689209
       },
       {
           "heading": 0.0,
           "lifetime_seconds": 0.0,
           "track_id": 2,
           "velocity": 0.0,
           "x_range": -4.0150322914123535,
           "y_range": 19.060630798339844
       },
       ...
   ],
   "roi_targets": [
       {
           "radar_target": {
               "aoa": -0.0010617646621540189,
               "range": 16.69966697692871,
               "rcs": 0.0,
               "target_ids": [
                   "sedan_monoDrive_01_5"
               ],
               "velocity": 0.3018724322319031
           },
           "roi_name": "test_right_roi"
       },
       ...
   ]

}
```

The JSON portion of the Radar return contains ground truth and actual detected 
Radar returns. All arrays in the returns are indexed by target number (i.e. 
index 0 of `aoas` corresponds with index 0 of `ranges` and so on.)

- **game_time:** The time in seconds since the simulation started when this sample was acquired
- **time:** The time in UTC seconds when this sample was acquired
- **message:** JSON containing the Radar return results
    - **gt_target_list:** JSON containing ground truth targets within the Radar range
        - **aoas:** Angle of arrival of each target in degrees
        - **ranges:** The range to each target in centimeters
        - **rcs:** The radial cross-section of the Radar return to each target in decibels per square meter (dBsm)
        - **target_ids:** The string representing the name of the actor that generated the target
        - **velocities:** The velocity in meters per second of each target
    - **target_list:** JSON containing the targets detected in the processed Radar signal
        - **aoas:** Angle of arrival of each target in degrees
        - **ranges:** The range to each target in centimeters
        - **rcs:** The radial cross-section of the Radar return to each target in decibels per square meter (dBsm)
        - **target_ids:** Array of target information for each target
            - **target_id:** The string representing the name of the actor that generated the target
        - **velocities:** The velocity in meters per second of each target
    - **tracks**: A list of all tracks current in the radar's field-of-view
        - **heading**: The current estimated heading of the track based on kinematics in radians
        - **lifetime_seconds**: The total number of seconds the track has been alive
        - **track_id**: The unique ID of this track
        - **velocity**: The current estimated velocity of the track based on kinematics in meters per second
        - **x_range**: The distance away from the radar in the x (lateral) direction (meters)
        - **y_range**: The distance away from the radar in the y (forward) direction (meters)
    - **roi_targets**: The list of current targets inside of the input ROIs
        - **radar_target**: The information about the target within the ROI (same as `target_list`)
        - **roi_name**: The unique ID assigned to the ROI from the configuration

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
