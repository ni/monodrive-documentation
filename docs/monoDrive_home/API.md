# Introduction

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


### Message Types

The following message types are currently defined for sending messages to the 
simulator:

| Message Type | Definition |
| ------------ | ---------- | 
| EgoVehicleConfig_ID | Configure the ego vehicle |
| EgoControl_ID | Control the ego vehicle |
| MapCommand_ID | Control the map |
| ScenarioConfig_ID | Configur the scenario |
| ScenarioInit_ID | Initializing the scenario |
| WaypointUpdate_ID | Update the current waypoint. |
| StreamData_ID | Control the data stream |
| SpawnActorCommand_ID | Spawn a new actor in the current scenario |
| UpdateActorCommand_ID | Update an existing actor in the current scenario |
| AttachSensorCommand_ID | Attach a new sensor to the ego vehicle |
| DetachSensorCommand_ID | Detach a sensor from the ego vehicle |
| StopAllSensorsCommand_ID | Stop all the sensors on the ego vehicle |
| StartAllSensorsCommand_ID | Start all the sensors on the ego vehicle |
| ActivateLicense | Activate the server license |
| WeatherConfig | Reconfigure a sensor while the simulator is running |
| REPLAY_ReConfigureSensorCommand_ID | Configure the current weather in the scenario |
| REPLAY_ConfigureSensorsCommand_ID | Replay the current sensor configuration |
| REPLAY_ConfigureTrajectoryCommand_ID | Replay the current trajectory configuration |
| REPLAY_StepSimulationCommand_ID | Replay the current step in the simulation |
| REPLAY_StateStepSimulationCommand_ID | Replay the current state in the simulation |
| GetMap | Get the currently loaded map information | 


## Receiving a Message

The monoDrive Simulator will send messages out over the simulator port 
(default `8999`) from either localhost or the IP address of the simulator if 
running in a networked mode. If a `listenport` is specified in the message, 
(e.g. a sensor that should stream data over a specified port), the message will
be sent to the specified port.

Each message will begin with a 12 byte monoDrive header starting with the 
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
