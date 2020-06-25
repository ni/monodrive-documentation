# IMU

The monoDrive IMU sensor reports the ego vehicle's acceleration in x, y and z 
axis, as well as the angular velocity in the same axis. The location and 
orientation of the sensor can be modified in the "x", "y" and "z" axis with 
respect to the origin of the ego vehicle.

## Configuration

```json
{
    "type": "IMU",
    "listen_port": 0,
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

The total sensor output is 51 bytes, where the first 16 bytes correspond to the 
monoDrive sensor header and the remaining 35 conform to the 3DM-GX2 Data 
Communications Protocol. The format for the protocol can be found 
[here](http://files.microstrain.com/dcp/Inertia-Link-3DM-GX2-data-communications-protocol.pdf). 
Below is a table of each element in the message:

| Byte  | Description   |
| ------------ | ------------ |
|Byte 1  | Start Byte |
|Bytes 2-5 | x acceleration bits represented as a float in single precision IEEE-754 format |
|Bytes 6-9 | y acceleration bits represented as a float in single precision IEEE-754 format |
|Byte 10-13 | z acceleration bits represented as a float in single precision IEEE-754 format |
|Bytes 14-17 | x angular velocity bits represented as a float in single precision IEEE-754 format |
|Bytes 18-21 | y angular velocity bits represented as a float in single precision IEEE-754 format |
|Bytes 22-25 | z angular velocity bits represented as a float in single precision IEEE-754 format |
|Bytes 26-29 | Timestamp as indicated in the above link |
|Bytes 20-31 | Checksum |
|Byte 32-35| Time of week |

<p>&nbsp;</p>