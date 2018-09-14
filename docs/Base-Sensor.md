### Parameter Format

Every sensor follows the following format as the base for what you can do with a sensor. The individual sensor pages will explain their own parameters in addition to these base parameters. 

```
{
      "type": string,
      "id": string,
      "packet_size": int,
      "listen_port": int,
      "display_process": bool,
      "sensor_process": bool,
      "location": {
        "x": float,
        "y": float,
        "z": float
      },
      "rotation": {
        "pitch": float,
        "yaw": float,
        "roll": float
      },
      "fps": float
}
```

- **type**: The values of type can be: `MultiCamera`, `Camera`, `Semantic`, `Lidar`, `IMU`, `GPS`, `RPM`, `Radar`, `Waypoint`, or `BoundingBox` depending on what type of sensor you are trying to set up.
- **id**: This value can be any string, but needs to be unique for that sensor.
- **packet_size**: An integer that is the packet size of the UDP packet to be transferred from the simulator to the client.
- **listen_port**: The TCP or UDP port that the simulator must send the data through for that sensor to get the data to the client.
- **display_process**: Should the client create a process to display the sensors data.
- **sensor_process**: Should the client create a process for the sensor to read data coming from the simulator.
- **location**: The relative location for which the sensor will be placed on the vehicle.
  - *x*: The x position.
  - *y*: The y position.
  - *z*: The z position.
- **rotation**: The relative rotation the sensor will have on the vehicle.
  - *pitch*: The pitch.
  - *yaw*: The yaw.
  - *roll*: The roll.
- **fps**: The number of times the sensor will sample per second.

### How to listen for sensor data 

1. Getting a reference to a sensor
- Use the `sensors` list on the `BaseVehicle` class to access a list of all the sensor instances
- Each sensor has a `type` attribute that is defined in the config, (Waypoint, Camera, GPS, etc.)
- Each sensor has a `name` attribute that follows this format <Type_Port>, this is to get a specific sensor
  - I.E. If a sensor's type is `Camera` and it's port is `8086` it's name attribute will be `Camera_8086`

2. Listening to a sensor's data queue
- Each sensor has two data queues and methods that provide each frame of data. One queue is intended for data processing while the other is intended for display purposes. 
- Calling `get_message()` or `get_dislpay_message()` on the sensor instance will return the most up to date data frame for that sensor from the simulator
- Calling `get_messages()` or `get_display_messages()` will return a list of data frames that are currently available in the sensor queues (the newest frame being the first in the list)

Example:
1. Get the single Waypoint sensor based on the sensor's `type` attribute, getting this sensor based on `type` is fine in this instance because there is only one Waypoint sensor in the vehicle configuration
2. Get a unique Camera sensor using the sensor's `name` attribute, using the `name` attribute is necessary in this instance because most configurations will have multiple cameras, using the `name` attribute will get a specific Camera

```
def plan_control_data(self, vehicle):
  waypoint = None
  camera = None
  for sensor in vehicle.sensors:
      if sensor.type == 'Waypoint':
          waypoint = sensor
      if sensor.name == 'Camera_8086':
          camera = sensor

  data_waypoint = waypoint.get_message()
  data_camera = camera.get_message()
  # The data in data_waypoint and data_camera is a dictionary.
  # For example: speed = data_waypoint['speed']
  # More information on what keys for the dictionary on the individual sensor pages.
```