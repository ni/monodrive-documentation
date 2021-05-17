# RSBR  

RSBR Description

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
    "num_threads": 8,
    "resolution": 5,
    "max_returns": 100000,
    "raycasts_per_thread": 100000,
    "max_targets_capture": 1000
}
```

 - **azi_fov:** The angle in the x-y plane to detect objects in degrees.
 - **ele_fov:** The angle in the y-z plane to detect objects in degrees.
 - **scan_distance:** The maximum distance the sensor will detect objects in meters.
 - **debug_draw:** If true, the oriented bounding boxes for each actor in the output will be drawn.
 - **num_threads:** Description.
 - **resolution:** Description.
 - **max_returns:** Description.
 - **raycasts_per_thread:** Description.
 - **max_targets_capture:** Description.

## Raw Output

Explanation

Chart if needed:

| column1   | column2 |
| ------------ | ------------ |
|Description 1 | Description 2 |

