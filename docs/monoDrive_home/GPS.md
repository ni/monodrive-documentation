# GPS

Provides GPS coordinates for the sensor's location. The coordinates are based
on the GPS anchor set for the current map.

## Configuration 

```
[
   {
    "type": "GPS",
    "listen_port": 8400,
    "location": {
      "x": -75.0,
      "y": -25.0,
      "z": 245.0
    },
    "rotation": {
      "pitch": 0.0,
      "yaw": 0.0,
      "roll": 0.0
    }
  }
]
```

The GPS sensor provides the location information from the sensor's location. 
The location and orientation are relative to the EGO vehicle's origin. 

## Raw Output

The total sensor output is 78 bytes, where the first 12 bytes correspond to the 
monoDrive sensor header and the remaining 66 conform to the Swift Navigation 
Binary Protocol. The format for the Swift Protocol can be found 
[here](https://github.com/swift-nav/libsbp/raw/master/docs/sbp.pdf). Below
is a table of each element in the message:

| Type  | Name   |
| ------------ | ------------ |
|Byte 0-1  | preamble |
|Bytes 1-3 | MSG_POS_LLH  |
|Byte 5-6 | Hash of the sensor id  |
|Byte 5-6  |  Length of the payload |
|Bytes 6-14 | Latitude bits represented as a double in double precision IEEE-754 format. |
|Bytes 14-22 | Longitude bits represented as a double in double precision IEEE-754 format  |
|Bytes 22-30 | Elevation bits represented as a double in double precision IEEE-754 format  |
|Bytes 30-38 | WorldLocation_x |
|Bytes 38-46 | WorldLocation_y|
|Bytes 36-40 | forward_x   |
|Bytes 40-44 | forward_y   |
|Bytes 44-48 | forward_z   |
|Bytes 48-52 | ego_yaw  |
|Bytes 52-56 | ego_speed |
|Bytes 56-58 | horizontal_acceleration  |
|Bytes 58-60 | vertical_acceleration  |
|Bytes 60-62  | Number of satellites used for signal  |
|Bytes 62-64  | Fixed more status|
|Bytes 64-66 | CRC  |

<p>&nbsp;</p>

