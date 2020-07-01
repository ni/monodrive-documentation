# State

The state sensor is created during a simulation to  output state information for 
actors with the `"desired_tags"` and cull out information of actors with the 
`"undesired_tags"`. This information can be serialized to a monoDrive Trajectory 
File and played back in Replay Mode.

## Configuration

``` json
[
 {
   "type": "State",
   "listen_port": 8700,
   "rotation": {
      "pitch": 0.0,
      "roll": 0.0,
      "yaw": 0.0
   },
      "location": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0
   },
   "desired_tags": [
      "vehicle"
   ],
   "undesired_tags": [
      "static"
   ],
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

``` json
[
    {
        "frame": {
            "objects": [
                {
                    "name": "Misc_TrafficCone_2",
                    "odometry": {
                        "angular_velocity": {
                            "x": null,
                            "y": 0.0,
                            "z": 0.0
                        },
                        "linear_velocity": {
                            "x": 0.0,
                            "y": 0.0,
                            "z": 0.0
                        },
                        "pose": {
                            "orientation": {
                                "w": 1.0,
                                "x": 0.0,
                                "y": 0.0,
                                "z": 0.0
                            },
                            "position": {
                                "x": 12200.0,
                                "y": 3710.0,
                                "z": 10.0
                            }
                        }
                    },
                    "oriented_bounding_box": [
                        {
                            "center": {
                                "x": 12199.9521484375,
                                "y": 3710.02758789062,
                                "z": 37.2839050292969
                            },
                            "extents": {
                                "x": 32.3424224853516,
                                "y": 32.3816871643066,
                                "z": 64.438591003418
                            },
                            "name": "None",
                            "orientation": {
                                "w": 1.0,
                                "x": 0.0,
                                "y": 0.0,
                                "z": 0.0
                            },
                            "scale": {
                                "x": 1.0,
                                "y": 1.0,
                                "z": 1.0
                            }
                        }
                    ],
                    "tags": [
                        "cone"
                    ]
                }
            ],
            "vehicles": [
                {
                    "state": {
                        "name": "compact_monoDrive_01_2",
                        "odometry": {
                            "angular_velocity": {
                                "x": 0.176449194550514,
                                "y": 0.0175474192947149,
                                "z": -0.517025172710419
                            },
                            "linear_velocity": {
                                "x": -1072.10705566406,
                                "y": 102.813850402832,
                                "z": -3.0032639503479
                            },
                            "pose": {
                                "orientation": {
                                    "w": 0.0343129225075245,
                                    "x": -0.000261345121543854,
                                    "y": -0.0231100562959909,
                                    "z": 0.999143958091736
                                },
                                "position": {
                                    "x": 8302.064453125,
                                    "y": 4282.83154296875,
                                    "z": 6.68744659423828
                                }
                            }
                        },
                        "oriented_bounding_box": [
                            {
                                "center": {
                                    "x": 8301.8271484375,
                                    "y": 4278.1474609375,
                                    "z": 96.9822463989258
                                },
                                "extents": {
                                    "x": 180.120330810547,
                                    "y": 141.698760986328,
                                    "z": 415.024993896484
                                },
                                "name": "Body",
                                "orientation": {
                                    "w": 0.508013129234314,
                                    "x": 0.53005838394165,
                                    "y": 0.467878460884094,
                                    "z": 0.49198642373085
                                },
                                "scale": {
                                    "x": 1.0,
                                    "y": 1.0,
                                    "z": 1.0
                                }
                            }
                        ],
                        "tags": [
                            "vehicle",
                            "dynamic",
                            "car",
                            "ego"
                        ]
                    },
                    "wheels": [
                        {
                            "id": 0,
                            "pose": {
                                "orientation": {
                                    "w": -1.0,
                                    "x": -2.42143833872888e-08,
                                    "y": -4.05416322735164e-08,
                                    "z": -2.23517382380578e-08
                                },
                                "position": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "z": 0.0
                                }
                            },
                            "speed": 35.477783203125
                        },
                        {
                            "id": 1,
                            "pose": {
                                "orientation": {
                                    "w": -0.42500364780426,
                                    "x": -0.062142189592123,
                                    "y": -0.902581810951233,
                                    "z": 0.0292612351477146
                                },
                                "position": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "z": 0.0
                                }
                            },
                            "speed": 38.278190612793
                        },
                        {
                            "id": 2,
                            "pose": {
                                "orientation": {
                                    "w": -0.208940535783768,
                                    "x": -0.0615476667881012,
                                    "y": -0.975900650024414,
                                    "z": 0.0131773687899113
                                },
                                "position": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "z": 0.0
                                }
                            },
                            "speed": 35.0245780944824
                        },
                        {
                            "id": 3,
                            "pose": {
                                "orientation": {
                                    "w": -0.468537330627441,
                                    "x": 0.0,
                                    "y": -0.883443713188171,
                                    "z": -8.88178419700125e-16
                                },
                                "position": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "z": 0.0
                                }
                            },
                            "speed": 37.7250938415527
                        }
                    ]
                },
                {
                    "state": {
                        "name": "subcompact_monoDrive_01_2",
                        "odometry": {
                            "angular_velocity": {
                                "x": 0.00397115619853139,
                                "y": -0.0178125314414501,
                                "z": -0.0100505687296391
                            },
                            "linear_velocity": {
                                "x": 1078.56713867188,
                                "y": 18.6435050964355,
                                "z": 0.526299476623535
                            },
                            "pose": {
                                "orientation": {
                                    "w": 0.999955952167511,
                                    "x": -0.00160277157556266,
                                    "y": 0.00162250676658005,
                                    "z": 0.00911359395831823
                                },
                                "position": {
                                    "x": 12218.37890625,
                                    "y": 5081.291015625,
                                    "z": 12.1331481933594
                                }
                            }
                        },
                        "oriented_bounding_box": [
                            {
                                "center": {
                                    "x": 12213.0517578125,
                                    "y": 5082.02734375,
                                    "z": 93.3432235717773
                                },
                                "extents": {
                                    "x": 164.623306274414,
                                    "y": 130.190704345703,
                                    "z": 251.728515625
                                },
                                "name": "Body",
                                "orientation": {
                                    "w": 0.509322762489319,
                                    "x": 0.5018350481987,
                                    "y": -0.492163270711899,
                                    "z": -0.496515303850174
                                },
                                "scale": {
                                    "x": 1.0,
                                    "y": 1.0,
                                    "z": 1.0
                                }
                            }
                        ],
                        "tags": [
                            "vehicle",
                            "dynamic",
                            "car"
                        ]
                    },
                    "wheels": [
                        {
                            "id": 0,
                            "pose": {
                                "orientation": {
                                    "w": 1.0,
                                    "x": -2.42835085373372e-10,
                                    "y": -3.94384187529795e-08,
                                    "z": 0.0
                                },
                                "position": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "z": 0.0
                                }
                            },
                            "speed": 36.0967025756836
                        },
                        {
                            "id": 1,
                            "pose": {
                                "orientation": {
                                    "w": -0.379620462656021,
                                    "x": -0.000789966084994376,
                                    "y": -0.925141930580139,
                                    "z": 0.00032415275927633
                                },
                                "position": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "z": 0.0
                                }
                            },
                            "speed": 36.1465301513672
                        },
                        {
                            "id": 2,
                            "pose": {
                                "orientation": {
                                    "w": -0.348176896572113,
                                    "x": -0.000799307250417769,
                                    "y": -0.937428414821625,
                                    "z": 0.000296876329230145
                                },
                                "position": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "z": 0.0
                                }
                            },
                            "speed": 36.1463508605957
                        },
                        {
                            "id": 3,
                            "pose": {
                                "orientation": {
                                    "w": -0.424199938774109,
                                    "x": 1.7337804900785e-18,
                                    "y": -0.905568540096283,
                                    "z": 3.98393264653351e-16
                                },
                                "position": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "z": 0.0
                                }
                            },
                            "speed": 36.189338684082
                        }
                    ]
                },
                {
                    "state": {
                        "name": "SUV_monoDrive_01_2",
                        "odometry": {
                            "angular_velocity": {
                                "x": 0.0349516868591309,
                                "y": -0.00629166793078184,
                                "z": 0.0216597486287355
                            },
                            "linear_velocity": {
                                "x": 1078.73571777344,
                                "y": -4.44047498703003,
                                "z": 0.0467185191810131
                            },
                            "pose": {
                                "orientation": {
                                    "w": 0.999997079372406,
                                    "x": -0.00159215414896607,
                                    "y": 0.000595311226788908,
                                    "z": -0.0017311725532636
                                },
                                "position": {
                                    "x": 10308.51953125,
                                    "y": 5396.32763671875,
                                    "z": 10.8787307739258
                                }
                            }
                        },
                        "oriented_bounding_box": [
                            {
                                "center": {
                                    "x": 10308.2490234375,
                                    "y": 5396.82373046875,
                                    "z": 102.802200317383
                                },
                                "extents": {
                                    "x": 206.304290771484,
                                    "y": 133.491729736328,
                                    "z": 465.631927490234
                                },
                                "name": "Body",
                                "orientation": {
                                    "w": 0.501211106777191,
                                    "x": 0.495349526405334,
                                    "y": -0.500353097915649,
                                    "z": -0.503053665161133
                                },
                                "scale": {
                                    "x": 1.0,
                                    "y": 1.0,
                                    "z": 1.0
                                }
                            }
                        ],
                        "tags": [
                            "vehicle",
                            "dynamic",
                            "car"
                        ]
                    },
                    "wheels": [
                        {
                            "id": 0,
                            "pose": {
                                "orientation": {
                                    "w": 1.0,
                                    "x": -9.95896559663478e-11,
                                    "y": -5.30137569398903e-08,
                                    "z": -6.93835247034258e-08
                                },
                                "position": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "z": 0.0
                                }
                            },
                            "speed": 36.15625
                        },
                        {
                            "id": 1,
                            "pose": {
                                "orientation": {
                                    "w": -0.401167452335358,
                                    "x": 0.00281385076232255,
                                    "y": -0.915999531745911,
                                    "z": -0.00123234267812222
                                },
                                "position": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "z": 0.0
                                }
                            },
                            "speed": 36.0364761352539
                        },
                        {
                            "id": 2,
                            "pose": {
                                "orientation": {
                                    "w": -0.32726725935936,
                                    "x": 0.00291450833901763,
                                    "y": -0.944926738739014,
                                    "z": -0.00100941490381956
                                },
                                "position": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "z": 0.0
                                }
                            },
                            "speed": 36.2052955627441
                        },
                        {
                            "id": 3,
                            "pose": {
                                "orientation": {
                                    "w": -0.449046015739441,
                                    "x": 0.0,
                                    "y": -0.893508613109589,
                                    "z": 0.0
                                },
                                "position": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "z": 0.0
                                }
                            },
                            "speed": 36.099666595459
                        }
                    ]
                }
            ]
        },
        "game_time": 1.01402580738068,
        "sample_count": 1,
        "time": 1593614676
    }
]
```

- **frame:** An array of all actors' state information for a single simulation step
- **objects** An array of object actors' in the scene
   - **name:** The name of the actor in the scene
   - **odometry** 
      - **angular_velocity:** The angular velocity of the actor in radians/second
      - **linear_velocity:** The linear velocity of the actor in radians/second
      - **pose:** Determines the position and orientation of the actor
         - **orientation:** The rotation of the actor as a quaternion
         - **position:** The position of this actor in the scene in centimeters relative to the origin
   - **oriented_bounding_box:** The bounding box of the actor in the scene
      - **center:** The location of the center of the box in centimeters relative to the origin
      - **extents:** The extents of the box from the center in centimeters
      - **name:** The name of the object this box is bounding
      - **orientation:** The rotation of the box as a quaternion
      - **scale:** The scale of this box as a coefficient
   - **tags:** The actor tags of this actor
- **vehicles**
   - **state**
      - **name:** The name of the actor in the scene
      - **odometry**
         - **angular_velocity:** The angular velocity of the actor in radians/second
         - **linear_velocity:** The linear velocity of the actor in radians/second
         - **pose:** Determines the position and orientation of the vehicle actor
            - **orientation:** The rotation of the actor as a quaternion
            - **position:** The position of this actor in the scene in centimeters relative to the origin
      - **oriented_bounding_box:** The bounding box of the actor in the scene
         - **center:** The location of the center of the box in centimeters relative to the origin
         - **extents:** The extents of the box from the center in centimeters
         - **name:** The name of the object this box is bounding
         - **orientation:** The rotation of the box as a quaternion
         - **scale:** The scale of this box as a coefficient
      - **tags:** The actor tags of this actor
      - **wheels:** Array of orientation information for each wheel as quaternions.
         - **id:**
         - **pose:** Determines the position and orientation of the wheel actor
            - **orientation:** The rotation of the wheels as a quaternion
            - **position:** The position of the wheels in the scene in centimeters relative to the origin
         - **speed:** The angular velocity of each of this actor's wheels in radians/second. Index 0, 1, 2, 3 correspond to front-left, front-right, rear-left, rear-right respectively
- **game_time** Time since the simulation started
- **sample_count**
- **time** The time in UTC seconds when this sample was acquired