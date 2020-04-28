# State

## State
The state sensor can be attached anywhere in the scene (preferably on the EGO 
vehicle) to record state information for actors with the `"desired_tags"` and
not record information of actors with the `"undesired_tags"`.

```
[
 {
    "type": "State",
    "listen_port": 8700,
    "packet_size": 1472,
    "desired_tags": [
      "vehicle"
    ],
    "undesired_tags": [
      "static"
    ]
    "debug_drawing": false,
		"include_obb": true
}
]
```

## Configuration

### Configuration Tags
- **desired_tags:** An array of actor tags to output with the state sensor. Any actor with these tags (and no tags in the `undesired_tags` array) will be output.
- **undesired_tags:** An array of actor tags to exclude from output. Any actor with one of these tags will not be incluuded.
- **debug_drawing:** If true, and `include_obb` is true, the oriented bounding boxes for each actor in the output will be drawn.
- **include_obb:** If true, then oriented bounding box information will be included for every actor.


### Raw Output Example
```
{
    "frame": [
        {
            "angular_velocity": [
                -0.0037721325643360615,
                0.00136165926232934,
                0.00760695431381464
            ],
            "brake_input": 0.0,
            "name": "EgoVehicle_0",
            "orientation": [
                -0.00021424639271572232,
                -0.0006922308821231127,
                0.0010298348497599363,
                0.9999992251396179
            ],
            "oriented_bounding_box": [
                {
                    "center": [
                        11533.7939453125,
                        131.47654724121094,
                        100.92992401123047
                    ],
                    "extents": [
                        197.2131805419922,
                        144.67471313476562,
                        485.64129638671875
                    ],
                    "name": "Body",
                    "orientation": [
                        0.49950170516967773,
                        -0.4981778562068939,
                        -0.5003031492233276,
                        0.5020096302032471
                    ],
                    "scale": [
                        1.0,
                        1.0,
                        1.0
                    ]
                }
            ],
            "position": [
                11533.7138671875,
                130.96014404296875,
                9.822959899902344
            ],
            "tags": [
                "vehicle",
                "dynamic",
                "car",
                "ego"
            ],
            "throttle_input": 0.27544909715652466,
            "velocity": [
                139.93212890625,
                0.04096534475684166,
                0.08481724560260773
            ],
            "wheel_input": 0.0,
            "wheel_speed": [
                3.648573398590088,
                6.440979957580566,
                3.671414613723755,
                6.538351058959961
            ],
            "wheels": [
                {
                    "id": 0,
                    "orientation": [
                        2.8535396268125623e-11,
                        1.330271004462702e-08,
                        3.4924596548080444e-10,
                        1.0
                    ]
                },
                {
                    "id": 1,
                    "orientation": [
                        0.0,
                        0.9949015974998474,
                        -4.40619762898109e-16,
                        -0.10085028409957886
                    ]
                },
                {
                    "id": 2,
                    "orientation": [
                        0.0,
                        0.9992740750312805,
                        -4.432218481120742e-16,
                        -0.03809463977813721
                    ]
                },
                {
                    "id": 3,
                    "orientation": [
                        0.0,
                        -0.9634871482849121,
                        4.3021142204224816e-16,
                        0.26775452494621277
                    ]
                }
            ]
        },
        {
            "angular_velocity": [
                9.81187986326404e-06,
                0.000777970883063972,
                0.0033688489347696304
            ],
            "brake_input": 0.0,
            "name": "sedan_monoDrive_02_C_12",
            "orientation": [
                -0.0002343906817259267,
                -0.0005387877463363111,
                -0.6983885169029236,
                0.7157185673713684
            ],
            "oriented_bounding_box": [
                {
                    "center": [
                        23782.91796875,
                        -7284.94091796875,
                        88.38402557373047
                    ],
                    "extents": [
                        189.29913330078125,
                        132.10438537597656,
                        510.71820068359375
                    ],
                    "name": "Body",
                    "orientation": [
                        0.008845925331115723,
                        -0.7030364274978638,
                        -0.7110462784767151,
                        0.00865030288696289
                    ],
                    "scale": [
                        1.0,
                        1.0,
                        1.0
                    ]
                }
            ],
            "position": [
                23782.986328125,
                -7285.568359375,
                5.163909912109375
            ],
            "tags": [
                "vehicle",
                "dynamic",
                "car",
                "traffic"
            ],
            "throttle_input": 0.6338397860527039,
            "velocity": [
                33.27668380737305,
                -1341.6343994140625,
                -0.0028500943444669247
            ],
            "wheel_input": 0.0011427635326981544,
            "wheel_speed": [
                45.14750671386719,
                45.12545394897461,
                45.27882766723633,
                45.258949279785156
            ],
            "wheels": [
                {
                    "id": 0,
                    "orientation": [
                        -3.844616003334522e-08,
                        -7.930793799459934e-09,
                        3.5762786865234375e-07,
                        1.0
                    ]
                },
                {
                    "id": 1,
                    "orientation": [
                        4.254921805113554e-05,
                        -0.12133453041315079,
                        0.0003480859741102904,
                        0.9926115870475769
                    ]
                },
                {
                    "id": 2,
                    "orientation": [
                        3.9882415876490995e-05,
                        -0.1136784553527832,
                        0.00034856103593483567,
                        0.9935175180435181
                    ]
                },
                {
                    "id": 3,
                    "orientation": [
                        1.7763568394002505e-15,
                        -0.01694026030600071,
                        0.0,
                        0.9998565316200256
                    ]
                }
            ]
        }
    ],
    "game_time": 75.87316131591797,
    "sample_count": 183,
    "time": 1588113012
}
```

- **angular_velocity:** The angular velocity of the actor in radians/second
- **brake_input:** The amount of brake being applied to the actor in floating point percentage 0 - 1.0
- **name:** The name of the actor in the scene
- **orientation:** The rotation of the actor as a quaternion
- **oriented_bounding_box:** The bounding box of the actor in the scene
    - **center:** The location of the center of the box in centimeters relative to the origin
    - **extents:** The extents of the box from the center, in centimeters
    - **name:** The name of the object this box is bounding
		- **orientation:** The rotation of the box as a quaternion
		- **scale:** The scale of this box as a coefficient
- **position:** The position of this actor in the scene in centimeters relative to the origin
- **tags:** The actor tags of this actor
- **velocity:** The current velocity of the actor in centimeters/second
- **wheel_input:** The percentatge of steering being applied to this actor's steering wheel from -1.0 to 1.0
- **wheel_speed:** The angular velocity of each of this actor's wheels in radians/second. Index 0, 1, 2, 3 correspond to front-left, front-right, rear-left, rear-right respectively
- **wheels:** Array of orientation information for each wheel as quaternions.

<p>&nbsp;</p>