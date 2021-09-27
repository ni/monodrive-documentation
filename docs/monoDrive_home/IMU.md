# IMU

The monoDrive IMU sensor reports the ego vehicle's acceleration in x, y and z 
axis, as well as the angular velocity in the same axis. The location and 
orientation of the sensor can be modified in the "x", "y" and "z" axis with 
respect to the origin of the ego vehicle.

## Configuration

```json
{
    "type": "IMU",
    "listen_port": 8500,
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

## Output

The total sensor output is 184 bytes, where the first 16 bytes correspond to the 
monoDrive sensor header and the remaining 168 conform to the IMU data output

Note that the simulator returns **acceleration** relative to the 
**global coordinate frame** and **angular velocity** relative to the 
**vehicle coordinate frame**. Below is a table of each element in the message:

| Type  | Name   | Units   |
| ------------ | ------------ | |
|Vector (Float) | Acceleration (x, y, z)| cm/s^2|
|Vector (Float) | Angular velocity (x, y, z) | radians/s|
|Vector (Float) | Angular acceleration (x, y, z) | radians/s^2|
|Vector (Float) | Velocity (x, y, z) |cm/s|
|Vector (Float) | Local acceleration (x, y, z) |cm/s^2 |
|Vector (Float) | Local angular_velocity (x, y, z) |radians/s| |
|Vector (Float) | Local angular_acceleration (x, y, z)|radians/s^2 |
|Vector (Float) | Local velocity (x, y, z) |cm/s |
|Vector (Float) | Position (x, y, z) | |
|Vector (Float) | Orientation (x, y, z, w) | |
|Vector (Float) | Parent position (x, y, z)| |
|Vector (Float) | Parent orientation (x, y, z, w)| |

<p>&nbsp;</p>