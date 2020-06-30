# Collision

Provides information about a collision for the ego vehicle.

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