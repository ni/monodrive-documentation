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

Explanation

Chart if needed:

| column1   | column2 |
| ------------ | ------------ |
|Description 1 | Description 2 |

