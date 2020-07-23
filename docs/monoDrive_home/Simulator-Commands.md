# Simulator API Commands

## Get Simulator Version Command
Retrieves the current version of the simulator and of the simulator API with the commands documented on this page.

**command ID**:  "GetVersion"

**command data**:
```json
{	
}
```
**response**: (string)

`simulator_version: 1.12, api_version: 4.0`


## Simulator Configuration Command
Used to configure a simulator session. This command includes the settings associated with the simulator session following the execution of the command and applies to all future sessions until the command is run again. The "pys_materials" settings affect the returns from lidar and radar sensors for the list of known materials in the scene.

**command ID**: “SimulatorConfig_ID”

**command data**:

```json
{
	"id": (string) - used to manage configurations on the client side,
	"server_ip": (string) - the ip or host of the running simulator. This is a client side setting and is ignored by the simulator,
 	"server_port": (integer) - the port on which the simulator is listening for
			connections. The default port is 8999 but this can be changed by
			launching the simulator with the -port nnnn command line argument.
			This is a client side setting and is ignored by the simulator.,
	"simulation_mode": 0, 1, 2, or 3 (closed loop, replay, pxi (HIL), fixed-time closed loop),
	"map": (string) - the name of the map to use for this simulation run.,
	"phys_materials": {
        "MaterialName": {
        "specular_exponent": (float) - the specular exponent of the material,
        "specular_coefficient": (float) - the specular coefficient of the material,
			"diffuse_coefficient": (float) - the diffuse coefficient of the material,
			"dielectric_constant": (float) - the dielectric constant of the material,
			"roughness": (float) - a value between 0.0 and 1.0  corresponding to the roughness of the material
        }
	},
	"traffic_configuration": {
        "max_vehicles": (integer) - the maximum number of traffic vehicles at any given time during this simulation run,
        "min_desired_speed": (float) - the minimum speed for traffic vehicles in cm/s,
        "max_desired_speed": (float) - the maximum speed for traffic vehicles in cm/s,
        "spawn_leash_distance": (float) - distance from ego vehicle at which traffic vehicles are removed from the scene,
        "min_spawn_range": (float) - the minimum distance from the ego vehicle at which to spawn new traffic vehicles,
        "swarm_traffic": (boolean) - whether to spawn traffic vehicles as one large group or individually,
        "vertical_offset": (float) - an offset to apply to the Z direction when spawning traffic vehicles,
        "overtake_slower_vehicles": (float) - whether to overtake or follow vehicles with a slower desired speed
	}
}
```

**response**: (string)

`simulator configuration mode = (0, 1, or 2)`  if the configure command was successful, or `invalid map specified` if an invalid map is requested


## EGO Vehicle Control Command
Configures the ego vehicle used in the simulation run.

**command ID**:  "EgoControl_ID"
**command data**:
```json
{
	"forward_amount": (float) - value between 0.0 and 1.0 referring to the amount 
of throttle to apply,
	"right_amount": (float) - value between -1.0 and 1.0 referring to the wheel 
position. Positive values are to the right of center,
	"brake_amount": (float) - value between 0.0 and 1.0 referring to the amount
				of braking to apply,
	"drive_mode": (integer) - value indicating the drive mode: -1 = reverse, 
0 = neutral, 1 = drive	
}
```

**response**: (string)

`complete` if the control command was applied, or `no ego vehicle found` if the ego vehicle is not in the simulation.


## Weather Configuration Command
Configures the weather used in the simulation run. The profiles parameter is optional, and if present, the profiles are added or modified based on the supplied parameters for each profile.

**command ID**:  "WeatherConfig"

**command data**:
```json
{
	"set_profile": (string) - the ID of the weather profile to use,
	"profiles": [- optional: an array of weather profiles to add/modify
		{
			"id": (string), - the ID of this profile. This ID is used with the 
			"set_profile" key above
			"SunPolarAngle": (float) the sun polar angle in degrees,
			"SunAzimuthAngle": (float) - the sun azimuth angle in degrees,
			"SunBrightness": (float) - the brightness level,
			"SunDirectionalLightIntensity": (float) - the directional Intensity,
			"SunDirectionalLightColor": (FLinearColor) - the color of the sun directional light in RGBA values between 0.0 and 1.0
			{ 
				"R":(float), - the red component of the color
				"G": (float), - the green component of the color
				"B": (float), - the blue component of the color
				"A": (float) - the alpha value (transparency) of the color
			},
			"SunIndirectLightIntensity": (float) - the intensity of the indirect sunlight,
			"CloudOpacity": (float) - the cloud opacity,
			"HorizontFalloff": (float) - the horizontal fall-off value,
			"ZenithColor": (FLinearColor) - the zenith color,
			"HorizonColor": (FLinearColor) - the horizon color,
			"CloudColor": (FLinearColor) - the cloud color,
			"OverallSkyColor": (FLinearColor) - the overall sky color,
			"SkyLightIntensity": (float) - the intensity of the sky light,
			"SkyLightColor": (FLinearColor) - the sky light color,
			"bPrecipitation": (boolean) - whether to add precipitation,
			"PrecipitationType": (string) - precipitation type (currently only "Rain" is supported),
			"PrecipitationAmount": (float) - the amount of precipitation to add. The higher the number the more dense the precipitation,
			"PrecipitationAccumulation": (float) - the amount of precipitation that accumulates on the ground,
			"bWind": (boolean) - whether wind is present (affects precipitation),
			"WindIntensity": (float) - the intensity of the wind,
			"WindAngle": (float) - the angle of the wind, in degrees,
			"bOverrideCameraPostProcessParameters": (boolean) - whether to override the camera post processing with the parameters below,
			"CameraPostProcessParameters": 
			{
				"AutoExposureMethod": (string) - the luminance computation method (e.g. "Histogram"),
				"AutoExposureMinBrightness": (float) - auto-exposure minimum adaptation,
				"AutoExposureMaxBrightness": (float) - auto-exposure maximum adaptation,
				"AutoExposureBias": (float) - logarithmic adjustment of the exposure
			}
		},
		...  - additional profiles
	]
}
```

**response**: (string)

`DynamicWeather is not available` if the weather actor is missing from the map
`Weather configuration failed` if the weather profiles are invalid or mal-formed
`Weather profile is invalid or missing` if the profile ID is not found
`Weather configured to {profile_id}` if the weather was properly configured


## Sensor Configuration Command
Configures a sensor or set of sensors to use with the ego vehicle in the simulation run. The command can take a single sensor configuration, or an array of sensor configurations.

**command ID**:  "REPLAY_ConfigureSensorsCommand_ID"

**command data**:

sensor_config or

```json
[
	Sensor_config, ...
]
```

**response**: (string)

`complete` if the sensor was properly configured


## Sensor Reconfiguration Command
Reconfigures a previously configured sensor or set of sensors to use with the ego vehicle in the simulation run. The command takes a single sensor configuration.

**command ID**:  "REPLAY_ReConfigureSensorCommand_ID"

**command data**:

sensor_config

**response**: (string)

`complete` if the sensor was properly reconfigured


## Trajectory Configuration Command
Configures the simulation trajectory. The trajectory file determines the initial state of the simulation for closed loop mode, and a series of frames for replay/replay step mode. The frames provide the location/state of all actors in the simulation as a given point.

**command ID**:  "REPLAY_ConfigureTrajectoryCommand_ID"

**command data**:

trajectory

**response**: (string)

`complete` if the trajectory was properly applied


## Step the Simulation Command
Steps the simulation (either forward or backward) by the specified number of frames using the previously configured trajectory.

**command ID**:  "REPLAY_StepSimulationCommand_ID"

**command data**:
```json
{
	amount: (integer) - the number of steps to advance the simulation by
}
```
**response**: (string)

`complete` if the simulation was advanced properly

## State Step Simulation Command
Sets the simulation frame to the provided frame.

**command ID**:  "REPLAY_StateStepSimulationCommand_ID"

**command data**:

A state_frame object

**response**: (string)

`complete` if the simulation was advanced properly


## Get Start Points Command
Retrieves the set of starting locations for the current simulation map.

**command ID**:  "GetStartPoints"

**command data**:
```json
{
}
```

**response**: (json object)
``` json
{
	"type": [ "PIE" | "startPlayer", ... ],
	"locations": [ [x (float), y (float), z (float)],... ],
	"Rotations": [ [yaw (float), pitch (float), roll (float)], ... ]
} 
```

## Get Map Command
Retrieves the current road network in the requested format.

**command ID**:  "GetMap"

**command data**:
```json
{
	"format":  "geojson" | "opendrive" | "point_array",
	"coordinates": "world" | "gis", - UE4 world coordinates or GIS
	"gis_anchor": { 		- the reference point to use for GIS coordinates
		"x": (float) latitude, 
		"y": (float) longitude, 
		"z": (float) elevation
	},
	"orientation": (float) - rotation to apply,
	"point_delta": (float) - distance between lane center points in centimeters
}
```

**response**: (json object, xml document, or json array, depending on requested format)


## Import Map Command
Replaces the current road network with the supplied one.

**command ID**:  "ImportMap"

**command data**:
A monoDrive GeoJSON map document

**response**:(boolean)

`true` if the map was properly loaded, `false` otherwise


## Spawn Ego Vehicle Command
Spawns the EGO vehicle using the supplied configuration. This command applies only to closed loop simulations.

**command ID**:  "SpawnVehicleCommand_ID"

**command data**:
```json
{
	"vehicle_dynamics": "pysx" | "carsim", (default is PysX)
	"body": {	- the requested color/type of the vehicle
		"color": (string),
		"type": (string)
	},
	"name": (string), - an id to use for this vehicle. This id will appear in state sensor data
	"tags": [(string),...], - the list of tags to apply
	"position": (FVector), - the initial position 
	"orientation" : (FQuat or FRotator), - the initial orientation
	"velocity": (FVector), - the initial velocity
	"angular_velocity": (FVector) - the initial angular velocity,
	"wheels": [ - array of wheel configuration options
		"id": (integer), - the id of the wheel
		"orientation": (FQuat), - the orientation
	],
	"wheel_speed": [(float), ...] - the wheel speed for each wheel
}
```

**response**:  (string) 
One of:
- `Successfully spawned vehicle.`
- `Spawn vehicle is only available in closed loop.`
- `Failed to spawn vehicle.`


## Closed Loop Configuration Command
Configures the closed loop simulation.

**command ID**:  "ClosedLoopConfigCommand_ID"

**command data**:
A closed_loop configuration json

**response**: (string)
One of
- `Successfully configured scenario.`
- `Failed to configure scenario. No actors spawned.`


## Closed Loop Step Command
Steps the closed loop simulation.

**command ID**:  "ClosedLoopStepCommand_ID"

**command data**:
```json
{
	"time_step": (float) - the fixed time step delta to step
}
```

**response**: (boolean)

`true` if the map was properly loaded, `false` otherwise
