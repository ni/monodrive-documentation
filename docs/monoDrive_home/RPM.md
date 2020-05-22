# RPM 

The monoDrive sensor give you information on the Revolutions per second on the specified wheel. You can obtain information of the four wheels on the car (0-3). 

## RPM
Provides RPM stream
```
[
{
  "type": "RPM",
  "listen_port": 8600,
  "location": {
    "x": 0,
    "y": 0,
    "z": 0
  },
  "rotation": {
    "pitch": 0,
    "yaw": 0,
    "roll": 0
  },
  "wheelNumber": 3
}
]
```
<p>&nbsp;</p>

## Configuration

### Raw Output Data Format

| Byte   | Description |
| ------------ | ------------ |
|Bytes 0-4  | The wheel number for the RPM sensor. |
|Bytes 4-8 | The wheel speed in RPM for the specific RPM sensor.  |

<p>&nbsp;</p>