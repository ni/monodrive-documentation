# Control Channel API Commands

## Introduction

The monoDrive Simulator and Scenario Editor support a robust Application 
Programming Interface (API) through a JSON messaging interface. The simulation
can be completely controlled by sending and receiving messages through TCP/IP
from any application that implements the API. 

monoDrive provides several example client applications in 
[C++](../../cpp_client/cpp_quick_start), 
[Python](../../python_client/quick_start), and 
[LabVIEW](../../LV_client/quick_start/LabVIEW_client_quick_start) that 
implement the monoDrive API. For more information on the Open Source monoDrive
clients, check out the overview [here](../../monodrive_clients).


## Sending a Message

A message to the monoDrive Simulator consists of a binary header followed by
the JSON string containing the message meta-data. The first part of the message
contains the header flag:

```
0x6d6f6e6f
```

followed by a 32-bit integer that states the length of the following JSON.

The JSON in the message is dependent on the type of message being sent and must
contain the following fields:

```json
{
  "type": <monoDrive message type>,
  "reference": <unique ID for this message>,
  "message": <JSON of the message>
}
```

- **type:** A string that describes the type of message being sent. 
- **reference:** An integer defining the unique ID from this message. 
- **message:** The actual JSON containing all the additional meta-data for the message type.


## Message Types

Used to configure a simulator session. This command includes the settings associated with the simulator session following the execution of the command and applies to all future sessions until the command is run again. The `pys_materials` settings affect the returns from lidar and radar sensors for the list of known materials in the scene.

The following message types are currently defined for sending messages to the simulator:

| Message Type | Definition |
| ------------ | ---------- | 
| [ClosedLoopConfigCommand_ID](#closed-loop-configuration) | Configures the closed loop simulation | 
| [ClosedLoopStepCommand_ID](#closed-loop-step) | Steps the closed loop simulation |
| [EgoControl_ID](#ego-vehicle-control) | Control the ego vehicle | 
| [GetMap](#get-map) | Get the currently loaded map information | 
| [GetStartPoints](#get-start-points) | Retrieves the set of starting locations |
| [GetVersion](#get-simulator-version) | Gets Current version |
| [ImportMap](#import-map) | Replaces the current road network |
| [REPLAY_ConfigureSensorsCommand_ID](#sensor-configuration) | Configure a sensor or set of sensors in a simulator run |
| [REPLAY_ConfigureTrajectoryCommand_ID](#trajectory-configuration) | Configures the simulation trajectory |
| [REPLAY_ReConfigureSensorCommand_ID](#sensor-reconfiguration) | Reconfigure a sensor or set of sensors in a simulation run |
| [REPLAY_StateStepSimulationCommand_ID](#state-step-simulation) | Steps the simulation |
| [REPLAY_StepSimulationCommand_ID](#step-the-simulation) | Steps the simulation |
| [SampleSensorsCommand_ID](#sample-command) | Obtain the latest sample from the sensors | 
| [SimulatorConfig_ID](#simulator-configuration) | Configure the Simulator session |
| [SpawnVehicleCommand_ID](#spawn-ego-vehicle) | Spawns the EGO vehicle | 
| [UpdateStateCommand_ID](#update-command) | Update the state of the ego and other actors in the scene| 
| [WeatherConfig](#weather-configuration) | Configure the current weather in the scenario | 

## Receiving a Message

The monoDrive Simulator will send messages out over the simulator port 
(default `8999`) from either localhost or the IP address of the simulator if 
running in a networked mode. If a `listen_port` is specified in the message, 
(e.g. a sensor that should stream data over a specified port), the message will
be sent to the specified port.

Each message will begin with a 16 byte monoDrive header starting with the 
header flag:

```
0x6f6e6f6d
```

and followed by a 32-bit integer specifying the length of the JSON message.

Each JSON message received from the simulator will contain, at a minimum, the
following fields:

```json
{
  "type": <monoDrive message type>,
  "reference": <unique ID for this message>,
  "message": <JSON of the message>
}
```

- **type:** A string that describes the type of message being sent. This will be the type specified by the message that elicited this response.
- **reference:** An integer defining the unique ID from this message. 
- **message:** The actual JSON containing all the additional meta-data for the message type.

## API Commands

### Closed Loop Configuration
Configures the closed loop simulation.

 - **Command ID:  "ClosedLoopConfigCommand_ID"**

 - **Command Data**:
A [closed_loop configuration](../../scenario_editor/scenarios) json

 - **Response**: (string)
	One of
	- `Successfully configured scenario.`
	- `Failed to configure scenario. No actors spawned.`


### Closed Loop Step
Steps the closed loop simulation.

 - **Command ID:  "ClosedLoopStepCommand_ID"**

 - **Command Data**:

	- *time_step*: (float) - the fixed time step delta to step

 - **Response**: (boolean)

	`true` if the map was properly loaded, `false` otherwise


## EGO Vehicle Control
Configures the ego vehicle used in the simulation run.

 - **Command ID:  "EgoControl_ID"**

 - **Command Data**:

	- *forward_amount*: (float) - value between 0.0 and 1.0 referring to the amount of throttle to apply
	- *right_amount*: (float) - value between -1.0 and 1.0 referring to the wheel position. Positive values are to the right of center
	- *brake_amount*: (float) - value between 0.0 and 1.0 referring to the amount of braking to apply
	- *drive_mode*: (integer) - value indicating the drive mode: -1 = reverse, 0 = neutral, 1 = drive	

 - **Response**: (string)

	`complete` if the control command was applied, or `no ego vehicle found` if the ego vehicle is not in the simulation.

## Get Map
Retrieves the current road network in the requested format.

 - **Command ID:  "GetMap"**

 - **Command Data**:
	- *format*:  "geojson" | "opendrive" | "point_array"
	- *coordinates*: "world" | "gis", - UE4 world coordinates or GIS
	- *gis_anchor*: the reference point to use for GIS coordinates
		- x: (float) latitude
		- y: (float) longitude
		- z: (float) elevation
	- *orientation*: (float) - rotation to apply
	- *point_delta*: (float) - distance between lane center points in centimeters

- **Response**: 

	json object, xml document, or json array, depending on requested format

## Get Start Points
Retrieves the set of starting locations for the current simulation map.

 - **Command ID:  "GetStartPoints"**

 - **Command Data**: `{}`

 - **Response**: (json object)
	- *type*: [ "PIE" | "startPlayer", ... ]
	- *locations*: [ [x (float), y (float), z (float)],... ]
	- *rotations*: [ [yaw (float), pitch (float), roll (float)], ... ]

## Get Simulator Version
Retrieves the current version of the simulator and of the simulator API with the commands documented on this page.

 - **Command ID:  "GetVersion"**

 - **Command Data**: `{}`

 - **Response**: (string)

	`simulator_version: 1.14, api_version: 5.0`


## Import Map
Replaces the current road network with the supplied one.

 - **Command ID:  "ImportMap"**

 - **Command Data**:
	A monoDrive [GeoJSON map document](../../scenario_editor/roads/#importing-and-exporting)

 - **Response**:(boolean)

	`true` if the map was properly loaded, `false` otherwise

## Sensor Configuration
Configures a sensor or set of sensors to use with the ego vehicle in the simulation run. The command can take a [single sensor configuration](Common.md), or an array of sensor configurations.

 - **Command ID:  "REPLAY_ConfigureSensorsCommand_ID"**

 - **Command Data**: [sensor_config](Common.md)

 - **Response**: (string)

	`complete` if the sensor was properly configured


## Trajectory Configuration
Configures the simulation [trajectory](../../scenario_editor/trajectory_files). The frames on this file provide the location/state of all actors in the simulation as a given point for replay/replay step mode. 

 - **Command ID:  "REPLAY_ConfigureTrajectoryCommand_ID"**

 - **Command Data**: [trajectory](../../scenario_editor/trajectory_files)

 - **Response**: (string)

	`complete` if the trajectory was properly applied

## Sample Command
Obtain the latest sample from the sensors.   

- **Command ID:  "SampleSensorsCommand_ID"**

- **Command Data**: Specify the amount of time to wait for the sensors to return the sample before they timeout.i.e `{"timeout":10000}`.

- **Response**: (string)
	- `"An attempt to sample sensors was made but no ego vehicle is registered to the simulator."` if for some reason an ego vehicle was not spawn.

## Sensor Reconfiguration
Reconfigures a previously configured sensor or set of sensors to use with the ego vehicle in the simulation run. The command takes a [single sensor configuration](Common.md).

 - **Command ID:  "REPLAY_ReConfigureSensorCommand_ID"**

 - **Command Data**: [sensor_config](Common.md)

 - **Response**: (string)

	`complete` if the sensor was properly reconfigured


## State Step Simulation
Sets the simulation frame to the provided frame.

 - **Command ID:  "REPLAY_StateStepSimulationCommand_ID"**

 - **Command Data**: [A state_frame object](../../scenario_editor/trajectory_files/#replaying-trajectory-files)

 - **Response**: (string)

	`complete` if the simulation was advanced properly


## Step the Simulation
Steps the simulation (either forward or backward) by the specified number of frames using the previously configured trajectory.

 - **Command ID:  "REPLAY_StepSimulationCommand_ID"**

 - **Command Data**:
	- *amount*: (integer) - the number of steps to advance the simulation by

 - **Response**: (string)

	`complete` if the simulation was advanced properly


## Simulator Configuration
Used to configure a simulator session. This command includes the settings associated with the simulator session following the execution of the command and applies to all future sessions until the command is run again. The "pys_materials" settings affect the returns from lidar and radar sensors for the list of known materials in the scene.

 - **Command ID: “SimulatorConfig_ID”**

 - **Command Data**: [simulator_config](Simulator-Configuration.md)

	- For more information on these values:
		- [*specular exponent*](https://en.wikipedia.org/wiki/Specular_highlight)
		- [*specular coefficient*](https://en.wikipedia.org/wiki/Specular_reflection)
		- [*diffuse coefficient*](https://en.wikipedia.org/wiki/Mass_diffusivity)
		- [*dielectric constant*](https://en.wikipedia.org/wiki/Relative_permittivity)

 - **Response**: (string)

	`simulator configuration mode = (0, 1, or 2)`  if the configure command was successful, or `invalid map specified` if an invalid map is requested


## Spawn Ego Vehicle
Spawns the EGO vehicle using the supplied configuration. This command applies only to closed loop simulations.

 - **Command ID:  "SpawnVehicleCommand_ID"**

 - **Command Data**: [vehicle_config](Vehicle-Configuration.md)
	- For more information on these values:
		- [Vehicle body type & color](Vehicle-Configuration.md)
		- [State sensor data](State-sensor.md)
		- [List of Tags](State-sensor.md)

- **Response**:  (string) One of:
	- `Successfully spawned vehicle.`
	- `Spawn vehicle is only available in closed loop.`
	- `Failed to spawn vehicle.`

## Update Command
Update the state of the ego and other actors in the scene based on the message.   

- **Command ID:**  **"UpdateStateCommand_ID"**

- **Command Data**: Frame information for the ego vehicle and other actors.

- **Response**: (string)
	- `complete` if the simulation was updated properly.

## Torque Command
Control vehicle applying torque and steering angle to each wheel on the vehicle.

- **Command ID:**  **"EgoTorqueControl_ID"**

- **Command Data**: String in JSON format containing:
	- Torque applied to brake for each wheel (N*cm)
	- Torque values for each wheel (N*cm)
	- Steering angle (radians)   
- **Response**: (string)
	- `complete` if the simulation was updated properly.


## Weather Configuration
Configures the weather used in the simulation run. The profiles parameter is optional, and if present, the profiles are added or modified based on the supplied parameters for each profile.

 - **Command ID:  "WeatherConfig"**

 - **Command Data**: [weather_config](Weather.md)

 - **Response**: (string)

	- `DynamicWeather is not available` if the weather actor is missing from the map
	- `Weather configuration failed` if the weather profiles are invalid or mal-formed
	- `Weather profile is invalid or missing` if the profile ID is not found
	- `Weather configured to {profile_id}` if the weather was properly configured

 &nbsp;