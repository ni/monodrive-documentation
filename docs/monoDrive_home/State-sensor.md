# State

The state sensor is created during a simulation to  output state information for 
actors with the `"desired_tags"` and cull out information of actors with the 
`"undesired_tags"`. This information can be serialized to a monoDrive Trajectory 
File and played back in Replay Mode.

## Configuration

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

- **desired_tags:** An array of actor tags to output with the state sensor. Any actor with these tags (and no tags in the `undesired_tags` array) will be output.
- **undesired_tags:** An array of actor tags to exclude from output. Any actor with one of these tags will not be included.
- **debug_drawing:** If true, and `include_obb` is true, the oriented bounding boxes for each actor in the output will be drawn.
- **include_obb:** If true, the oriented bounding box information will be included for every actor.


## Raw Output 

```
{
        "frame": {
            "objects": [],
            "vehicles": [
                {
                    "state": {
                        "name": "sedan_monoDrive_01_B",
                        "odometry": {
                            "angular_velocity": {
                                "x": 7.55819055484608e-05,
                                "y": 5.34055288881063e-06,
                                "z": -0.00065633503254503
                            },
                            "linear_velocity": {
                                "x": 470.542633056641,
                                "y": 12.6052408218384,
                                "z": 0.000564481131732464
                            },
                            "pose": {
                                "orientation": {
                                    "w": 0.999909520149231,
                                    "x": 5.96524696447887e-05,
                                    "y": 0.000277796178124845,
                                    "z": 0.0134492535144091
                                },
                                "position": {
                                    "x": -2410.50341796875,
                                    "y": 11095.525390625,
                                    "z": 18.4553375244141
                                }
                            }
                                },
                                "oriented_bounding_box": [{
                                        "center": {
                                                "x": -2413.9150390625,
                                                "y": 11095.55859375,
                                                "z": 92.6209335327148
                                        },
                                        "extents": {
                                                "x": 174.510543823242,
                                                "y": 122.631454467773,
                                                "z": 456.815185546875
                                        },
                                        "name": "Body",
                                        "orientation": {
                                                "w": 0.508010506629944,
                                                "x": 0.504297912120819,
                                                "y": -0.491889208555222,
                                                "z": -0.495634883642197
                                        },
                                        "scale": {
                                                "x": 1.0,
                                                "y": 1.0,
                                                "z": 1.0
                                        }
                                }],
                                "tags": ["vehicle", "dynamic", "car"]
                        },
                        "wheels": [
                {
                                "id": 0,
                                "pose": {
                                        "orientation": {
                                                "w": 1.0,
                                                "x": 1.25032784126233e-08,
                                                "y": -2.89329591396381e-08,
                                                "z": -1.86264514923096e-09
                                        },
                                        "position": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0
                                        }
                                },
                                "speed": 15.6376466751099
                        }, {
                                "id": 1,
                                "pose": {
                                        "orientation": {
                                                "w": -0.98364132642746,
                                                "x": -3.50356895069126e-05,
                                                "y": -0.180137902498245,
                                                "z": 0.000191312050446868
                                        },
                                        "position": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0
                                        }
                                },
                                "speed": 15.6419067382813
                        }, {
                                "id": 2,
                                "pose": {
                                        "orientation": {
                                                "w": 0.972333788871765,
                                                "x": -4.54223409178667e-05,
                                                "y": -0.233595669269562,
                                                "z": -0.000189068901818246
                                        },
                                        "position": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0
                                        }
                                },
                                "speed": 15.629693031311
                        }, {
                                "id": 3,
                                "pose": {
                                        "orientation": {
                                                "w": -0.9913330078125,
                                                "x": -4.71844785465692e-16,
                                                "y": -0.131373181939125,
                                                "z": 0.0
                                        },
                                        "position": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0
                                        }
                                },
                                "speed": 15.6336793899536
            }
        ]
        },         
        {
                        "state": {
                                "name": "sedan_monoDrive_01_C_1",
                                "odometry": {
                                        "angular_velocity": {
                                                "x": 0.00157507439143956,
                                                "y": -0.00409367913380265,
                                                "z": 0.0561014041304588
                                        },
                                        "linear_velocity": {
                                                "x": -299.226196289063,
                                                "y": 308.347076416016,
                                                "z": -0.00424314336851239
                                        },
                                        "pose": {
                                                "orientation": {
                                                        "w": 0.397562474012375,
                                                        "x": 0.0015277904458344,
                                                        "y": 0.000960768782533705,
                                                        "z": 0.917573273181915
                                                },
                                                "position": {
                                                        "x": 3521.85034179688,
                                                        "y": 6405.2099609375,
                                                        "z": 18.4525375366211
                                                }
                                        }
                                },
                                "oriented_bounding_box": [{
                                        "center": {
                                                "x": 3524.37670898438,
                                                "y": 6402.64111328125,
                                                "z": 92.6091003417969
                                        },
                                        "extents": {
                                                "x": 174.510543823242,
                                                "y": 122.631454467773,
                                                "z": 456.815185546875
                                        },
                                        "name": "Body",
                                        "orientation": {
                                                "w": 0.659827828407288,
                                                "x": 0.655851125717163,
                                                "y": 0.259662300348282,
                                                "z": 0.258963227272034
                                        },
                                        "scale": {
                                                "x": 1.0,
                                                "y": 1.0,
                                                "z": 1.0
                                        }
                                }],
                                "tags": ["vehicle", "dynamic", "car", "traffic"]
                        },
                        "wheels": [
                {
                                "id": 0,
                                "pose": {
                                        "orientation": {
                                                "w": -0.999999940395355,
                                                "x": 3.74275259673595e-08,
                                                "y": 4.21423465013504e-08,
                                                "z": 5.0663948059082e-07
                                        },
                                        "position": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0
                                        }
                                },
                                "speed": 14.6655864715576
                        }, {
                                "id": 1,
                                "pose": {
                                        "orientation": {
                                                "w": 0.561672270298004,
                                                "x": 0.0151973990723491,
                                                "y": -0.827155828475952,
                                                "z": 0.0103196483105421
                                        },
                                        "position": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0
                                        }
                                },
                                "speed": 14.3515720367432
                        }, {
                                "id": 2,
                                "pose": {
                                        "orientation": {
                                                "w": -0.216665610671043,
                                                "x": 0.0183354970067739,
                                                "y": -0.976065158843994,
                                                "z": -0.00407008826732635
                                        },
                                        "position": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0
                                        }
                                },
                                "speed": 14.7026586532593
                        }, {
                                "id": 3,
                                "pose": {
                                        "orientation": {
                                                "w": -0.0389213487505913,
                                                "x": 5.6843418860808e-14,
                                                "y": -0.99924224615097,
                                                "z": -1.77635683940025e-15
                                        },
                                        "position": {
                                                "x": 0.0,
                                                "y": 0.0,
                                                "z": 0.0
                                        }
                                },
                                "speed": 14.4331750869751
            }
            ]
                }
        ]
    },
        "game_time": 22.8891162872314,
        "sample_count": 255,
        "time": 1592005987
}
```

- **frame:** An array of all actors' state information for a single simulation step
- **angular_velocity:** The angular velocity of the actor in radians/second
- **brake_input:** The amount of brake being applied to the actor in floating point percentage 0 - 1.0
- **name:** The name of the actor in the scene
- **orientation:** The rotation of the actor as a quaternion
- **oriented_bounding_box:** The bounding box of the actor in the scene
    - **center:** The location of the center of the box in centimeters relative to the origin
    - **extents:** The extents of the box from the center in centimeters
    - **name:** The name of the object this box is bounding
		- **orientation:** The rotation of the box as a quaternion
		- **scale:** The scale of this box as a coefficient
- **position:** The position of this actor in the scene in centimeters relative to the origin
- **tags:** The actor tags of this actor
- **velocity:** The current velocity of the actor in centimeters/second
- **wheel_input:** The percentage of steering being applied to this actor's steering wheel from -1.0 to 1.0
- **wheel_speed:** The angular velocity of each of this actor's wheels in radians/second. Index 0, 1, 2, 3 correspond to front-left, front-right, rear-left, rear-right respectively
- **wheels:** Array of orientation information for each wheel as quaternions.
