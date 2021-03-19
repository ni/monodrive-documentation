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


## Message Types

Used to configure a simulator session. This command includes the settings associated with the simulator session following the execution of the command and applies to all future sessions until the command is run again. The `pys_materials` settings affect the returns from lidar and radar sensors for the list of known materials in the scene.

The following message types are currently defined for sending messages to the simulator:

| Message Type | Definition |
| ------------ | ---------- | 
| [ClosedLoopConfigCommand_ID](../Simulator-Commands/#closed-loop-configuration) | Configures the closed loop simulation | 
| [ClosedLoopStepCommand_ID](../Simulator-Commands/#closed-loop-step) | Steps the closed loop simulation |
| [EgoControl_ID](../Simulator-Commands/#ego-vehicle-control) | Control the ego vehicle | 
| [GetMap](../Simulator-Commands/#get-map) | Get the currently loaded map information | 
| [GetStartPoints](../Simulator-Commands/#get-start-points) | Retrieves the set of starting locations |
| [GetVersion](../Simulator-Commands/#get-simulator-version) | Gets Current version |
| [ImportMap](../Simulator-Commands/#import-map) | Replaces the current road network |
| [REPLAY_ConfigureSensorsCommand_ID](../Simulator-Commands/#sensor-configuration) | Configure a sensor or set of sensors in a simulator run |
| [REPLAY_ConfigureTrajectoryCommand_ID](../Simulator-Commands/#trajectory-configuration) | Configures the simulation trajectory |
| [REPLAY_ReConfigureSensorCommand_ID](../Simulator-Commands/#sensor-reconfiguration) | Reconfigure a sensor or set of sensors in a simulation run |
| [REPLAY_StateStepSimulationCommand_ID](../Simulator-Commands/#state-step-simulation) | Steps the simulation |
| [REPLAY_StepSimulationCommand_ID](../Simulator-Commands/#step-the-simulation) | Steps the simulation |
| [SimulatorConfig_ID](../Simulator-Commands/#simulator-configuration) | Configure the Simulator session |
| [SpawnVehicleCommand_ID](../Simulator-Commands/#spawn-ego-vehicle) | Spawns the EGO vehicle | 
| [WeatherConfig](../Simulator-Commands/#weather-configuration) | Configure the current weather in the scenario | 
| [SampleSensorsCommand_ID](../Simulator-Commands/#sample-command) | Obtain the latest sample from the sensors | 
| [UpdateStateCommand_ID](../Simulator-Commands/#update-command) | Update the state of the ego and other actors in the scene based on the message | 



## Receiving a Message

The monoDrive Simulator will send messages out over the simulator port 
(default `8999`) from either localhost or the IP address of the simulator if 
running in a networked mode. If a `listen_port` is specified in the message, 
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
