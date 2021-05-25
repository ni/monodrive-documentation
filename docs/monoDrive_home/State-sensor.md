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
                       {
                    "control_state": {
                        "lane_change_left": false,
                        "lane_change_right": false,
                        "lane_id": 1,
                        "road_id": 0,
                        "s": 5077.20947265625,
                        "section_id": 0
                    },
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
      - **linear_velocity:** The linear velocity of the actor in centimeter/second
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
- **vehicles** An array of vehicle actors' in the scene
   - **control_state** 
      - **lane_change_left:** The vehicle is considered to be in a left lane change.
      - **lane_change_right:** The vehicle is considered to be in a right lane change.
      - **lane_id:** The opendrive lane id, changes to new lane id as soon as the vehicle begins the lane change. 
      - **road_id:** The opendrive road id.
      - **s:** The opendrive distance along the road.
      - **section_id:** The opendrive section id along the lane.
   - **state**
      - **name:** The name of the actor in the scene
      - **odometry**
         - **angular_velocity:** The angular velocity of the actor in radians/second
         - **linear_velocity:** The linear velocity of the actor in centimeter/second
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
- **sample_count** Sample number since the sensor started streaming
- **time** The time in UTC seconds when this sample was acquired