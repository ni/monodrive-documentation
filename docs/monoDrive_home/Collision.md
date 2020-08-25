# Collision

Provides collision information between the ego vehicle and objects specified by the `desired_tags`. 

## Configuration

``` json
{
    "type": "Collision",
    "undesired_tags": [
        "static"
    ],
    "desired_tags": [
        "vt"
    ],
    "listen_port": 8800,
    "location": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "rotation": {
        "pitch": 0.0,
        "roll": 0.0,
        "yaw": 0.0
    }
}
```

## Raw Output

``` json
{
	"acceleration": {
		"x": 0.0,
		"y": 0.0,
		"z": 0.0
	},
	"brake_input": 0.0,
	"collision": true,
	"distance": 0.0,
	"forward_acceleration": 0.0,
	"forward_velocity": 0.0,
	"game_time": 15.1504135131836,
	"name": "EgoVehicle_0",
	"relative_velocity": {
		"x": 0.0,
		"y": 0.0,
		"z": 0.0
	},
	"sample_count": 106,
	"targets": [{
		"acceleration": {
			"x": 0.0,
			"y": 0.0,
			"z": 0.0
		},
		"brake_input": 0.0,
		"collision": false,
		"distance": 158.330791015625,
		"forward_acceleration": 0.0,
		"forward_velocity": 64.3631000976562,
		"name": "compact_monoDrive_01_8",
		"relative_velocity": {
			"x": -64.3631820678711,
			"y": 1.31770466396119e-05,
			"z": -7.13992267264985e-06
		},
		"throttle_input": 0.613833427429199,
		"time_to_collision": 3.40282346638529e+38,
		"velocity": {
			"x": 64.3631820678711,
			"y": -1.31770466396119e-05,
			"z": 7.13992267264985e-06
		},
		"wheel_input": -0.0034033739939332
	}],
	"throttle_input": 0.0,
	"time": 1589232687,
	"time_to_collision": 0.0,
	"velocity": {
		"x": 0.0,
		"y": 0.0,
		"z": 0.0
	},
	"wheel_input": 0.0
}
```

These values are part of the outer JSON and do not pertain to targets:

 - **game_time:** The current number of seconds since the simulation started.

 - **sample_count:** The total number of samples that have been collected since the sensor began sampling

 - **time:** The current epoch time


The targets array contains all the information for the target vehicle. The outer JSON also includes this same information for the ego vehicle:

 - **acceleration:** The vehicle acceleration in centimeters per second per second relative to the vehicle coordinates.

    - **x:** Acceleration in the vehicle x-direction

    - **y:** Acceleration in the vehicle y-direction

    - **z:** Acceleration in the vehicle z-direction

 - **brake_input:** The current value of the brake input from 0 to 1

 - **collision:** If true, this vehicle is currently colliding with something. For the ego vehicle, it is always colliding with itself and this value should be ignored.

 - **distance:** The distance from the tagged vehicle in centimeters. For the ego vehicle this is always 0.

 - **forward_acceleration:** The vehicle acceleration in centimeters per second relative to the vehicle's forward vector.

 - **forward_velocity:** The vehicle velocity in centimeters per second relative to the vehicle's forward vector.

 - **name:** The name of the vehicle in the simulation.

 - **relative_velocity:** The relative velocity in centimeters per second between the ego vehicle and the target vehicle. For the ego vehicle this is always 0.

    - **x:** The velocity in the x-direction.

    - **y:** The velocity in the y-direction.

    - **z:** The velocity in the z-direction.

 - **throttle_input:** The current throttle input to the ego vehicle from 0 to 1.

 - **time_to_collision:** The number of seconds until the ego vehicle collides with the target at the current velocity. For the ego vehicle this is always 0.

 - **velocity:** The current velocity of the vehicle in centimeters per second.

    - **x:** The velocity in the x-direction.

    - **y:** The velocity in the y-direction.

    - **z:** The velocity in the z-direction.

 - **wheel_input:** The current steering input to the ego vehicle from -1 to 1.

 <p>&nbsp;</p>