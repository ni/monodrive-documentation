# RSBR  

The Radar Shooting Bouncing Ray (RSBR) sensor provides ground truth hit point access to targets that are within the field-of-view of a simulated radar. The output of the sensor provides target intersection information at the resolution and field-of-view specified by the configuration. 

## Configuration

``` json
{
    "type": "RSBR",
    "listen_port": 0,
    "location": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "rotation": {
        "pitch": 0.0,
        "roll": 0.0,
        "yaw": 0.0
    },
    "azi_fov": 90,
    "ele_fov": 90,
    "scan_distance": 3000,
    "debug_draw": false,
    "num_threads": 8
}
```

 - **azi_fov:** The horizontal field-of-view along the azimuth of the sensor in degrees.
 - **ele_fov:** The vertical field-of-view along the elevation of the sensor in degrees.
 - **scan_distance:** The maximum distance the sensor will detect objects in centimeters.
 - **debug_draw:** If true, the rays that are cast will be drawn in editor mode.
 - **num_threads:** The total number of threads that will be used to compute the ray casts.

## Raw Output

The RSBR dataframe contains a vector of `RSBRHitPoint`s. Here is the information sent back for a single hit point.

- **distance:** The magnitude of the vector from the sensor to the target in meters.
- **dir_x:** The distance in the x-direction in centimeters.
- **dir_y:** The distance in the y-direction in centimeters.
- **dir_z:** The distance in the z-direction in centimeters.
- **radial_velocity:** The velocity along the radial direction of the Radar.
- **reflection:** The electrical intensity of the return for the Radar.
- **normal_x:** The x component of the unit normal vector.
- **normal_y:** The y component of the unit normal vector.
- **normal_z:** The z component of the unit normal vector.
- **dielectric_constant:** The coefficient of the material's dielectric property.
- **roughness:** The coefficient of the material's roughness property.

<p>&nbsp;</p>

<div class="img_container">
  <video width=650px height=238px class="border" muted autoplay loop>
    <source src="https://cdn.monodrive.io/readthedocs/rsbr.mp4" type="video/mp4">
  </video>
</div> 

<p>&nbsp;</p>