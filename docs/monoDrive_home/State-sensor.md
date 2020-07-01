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
        "frame": [
            {
                "angular_velocity": [
                    0.0,
                    0.0,
                    0.0
                ],
                "name": "ScenarioVehicle_1",
                "orientation": [
                    -0.0,
                    0.0,
                    -0.711412131786346,
                    0.702775061130524
                ],
                "position": [
                    15014.271484375,
                    15183.6748046875,
                    13.4014129638672
                ],
                "tags": [
                    "vehicle",
                    "dynamic",
                    "ego"
                ],
                "velocity": [
                    -16.3557872772217,
                    -1338.95532226562,
                    -106.103073120117
                ],
                "wheel_speed": [
                    37.5429077148438,
                    37.542911529541,
                    36.2453956604004,
                    36.2453651428223
                ],
                "wheels": [
                    {
                        "id": 1,
                        "orientation": [
                            0.000226454139919952,
                            -0.183446362614632,
                            -0.00121349352411926,
                            -0.983028948307037
                        ]
                    },
                    {
                        "id": 2,
                        "orientation": [
                            0.000226826639845967,
                            -0.183446362614632,
                            -0.0012154895812273,
                            -0.983028948307037
                        ]
                    }
                ]
            },
            {
                "angular_velocity": [
                    0.0,
                    0.0,
                    0.0
                ],
                "name": "ScenarioVehicle2",
                "orientation": [
                    0.0,
                    -0.0,
                    0.707279205322266,
                    0.706934332847595
                ],
                "position": [
                    13788.841796875,
                    15402.5810546875,
                    13.4014129638672
                ],
                "tags": [
                    "vehicle",
                    "dynamic"
                ],
                "velocity": [
                    -0.326673209667206,
                    669.6806640625,
                    -106.103073120117
                ],
                "wheel_speed": [
                    18.5416431427002,
                    18.5416488647461,
                    17.8418426513672,
                    17.8418998718262
                ],
                "wheels": [
                    {
                        "id": 1,
                        "orientation": [
                            -5.69175390410237e-05,
                            -0.0918043628334999,
                            0.000617369019892067,
                            -0.995776891708374
                        ]
                    },
                    {
                        "id": 2,
                        "orientation": [
                            -5.68706418562215e-05,
                            -0.0918043702840805,
                            0.000616860284935683,
                            -0.995776891708374
                        ]
                    }
                ]
            },
            {
                "angular_velocity": [
                    0.0,
                    0.0,
                    0.0
                ],
                "name": "ScenarioVehicle3",
                "orientation": [
                    -0.0,
                    0.0,
                    -0.699469923973083,
                    0.71466201543808
                ],
                "position": [
                    14685.7451171875,
                    13615.357421875,
                    13.4014129638672
                ],
                "tags": [
                    "vehicle",
                    "dynamic"
                ],
                "velocity": [
                    28.7678089141846,
                    -1338.74633789062,
                    -106.103073120117
                ],
                "wheel_speed": [
                    37.5427589416504,
                    37.5427627563477,
                    36.2454223632812,
                    36.2453994750977
                ],
                "wheels": [
                    {
                        "id": 1,
                        "orientation": [
                            -0.000931824557483196,
                            -0.183442145586014,
                            0.00499339820817113,
                            -0.983017385005951
                        ]
                    },
                    {
                        "id": 2,
                        "orientation": [
                            -0.000925570144318044,
                            -0.183442160487175,
                            0.00495988270267844,
                            -0.98301750421524
                        ]
                    }
                ]
            }
        ],
        "game_time": 14.0151948928833,
        "time": 1558625570
    }
]

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
