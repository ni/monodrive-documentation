# Common

## Parameter Format

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

## Configuration

- **type**: The values of type can be: `MultiCamera`, `Camera`, `Semantic`, `Lidar`, `IMU`, `GPS`, `RPM`, `Radar`, `Waypoint`, or `BoundingBox` depending on what type of sensor you are trying to set up.
- **listen_port**: The TCP or UDP port that the simulator must send the data through for that sensor to get the data to the client.
- **location**: The relative location for which the sensor will be placed on the vehicle.
  - *x*: The x position.
  - *y*: The y position.
  - *z*: The z position.
- **rotation**: The relative rotation the sensor will have on the vehicle.
  - *pitch*: The pitch.
  - *yaw*: The yaw.
  - *roll*: The roll.
