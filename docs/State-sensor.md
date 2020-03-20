## State Sensor

The monoDrive state sensor reports information of all objects included in the **desired_tags** list. 
It will ignore the objects in the **undesired_tags** list.  

#### Configuration
```
[
 {
    "type": "State",
    "listen_port": 8700,
    "packet_size": 1472,
    "debug_drawing": true,
    "desired_tags": [
      "vehicle"
    ],
  "undesired_tags": [
  "static"
  ]
}

]
```

### Raw Output Example
```
{
	"frame": [{
		"angular_velocity": [0.009866185486316681, 0.011342001147568226, 0.6717177629470825],
		"brake_input": 0.0,
		"name": "ScenarioVehicle_1",
		"orientation": [0.011697866953909397, -0.02252720296382904, -0.9042208790779114, 0.42631015181541443],
		"position": [15262.076171875, 17526.494140625, 6.538963317871094],
		"tags": ["vehicle", "dynamic", "ego"],
		"throttle_input": 0.0,
		"velocity": [-334.3295593261719, -541.6399536132813, 0.05538097396492958],
		"wheel_input": 0.0,
		"wheel_speed": [15.510863304138184, 23.817401885986328, 25.251663208007813, 19.639755249023438],
		"wheels": [{
			"id": 1,
			"orientation": [-0.026308560743927956, -0.18871694803237915, -0.13943514227867126, -0.97172611951828]
		}, {
			"id": 2,
			"orientation": [0.08424673974514008, 0.6621711254119873, 0.1512315273284912, -0.7290821671485901]
		}]
	}, {
		"angular_velocity": [0.0037936856970191, -0.0072604212909936905, -0.019885115325450897],
		"brake_input": 0.0,
		"name": "ScenarioVehicle2",
		"orientation": [-0.002732698805630207, 0.0007744110189378262, -0.7097418904304504, 0.7044562101364136],
		"position": [15386.1279296875, 9367.8935546875, 10.581443786621094],
		"tags": ["vehicle", "dynamic"],
		"throttle_input": 0.0,
		"velocity": [-12.40577220916748, -1294.0648193359375, 0.004561997950077057],
		"wheel_input": 0.0,
		"wheel_speed": [42.87668228149414, 42.90896987915039, 42.922122955322266, 42.861324310302734],
		"wheels": [{
			"id": 1,
			"orientation": [0.0028412831015884876, 0.659138023853302, 0.0014229556545615196, -0.752015233039856]
		}, {
			"id": 2,
			"orientation": [0.0014198796125128865, -0.9493798017501831, -0.0028250974137336016, -0.31411468982696533]
		}]
	}],
	"game_time": 4.010171890258789,
	"sample_count": 19,
	"time": 1558625081
}
```

