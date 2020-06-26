# GPS

The GPS coordinates are based on the GPS anchor set for the current map. The location and orientation are relative to the ego vehicle's origin.

## Configuration 

``` json
{
  "type": "GPS",
  "listen_port": 8400,
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

The total sensor output is 82 bytes, where the first 16 bytes correspond to the 
monoDrive sensor header and the remaining 66 conform to the Swift Navigation 
Binary Protocol. The format for the Swift Protocol can be found 
[here](https://github.com/swift-nav/libsbp/raw/master/docs/sbp.pdf). Below
is a table of each element in the message:

| Type  | Name   |
| ------------ | ------------ |
|Byte 1  | preamble |
|Bytes 2-3 | MSG_POS_LLH  |
|Byte 4-5 | Hash of the sensor id  |
|Byte 6  |  Length of the payload |
|Bytes 7-14 | Latitude bits represented as a double in double precision IEEE-754 format. |
|Bytes 15-22 | Longitude bits represented as a double in double precision IEEE-754 format  |
|Bytes 23-30 | Elevation bits represented as a double in double precision IEEE-754 format  |
|Bytes 31-34 | WorldLocation_x |
|Bytes 35-38 | WorldLocation_y|
|Bytes 39-42 | forward_x   |
|Bytes 43-46 | forward_y   |
|Bytes 47-50 | forward_z   |
|Bytes 51-54 | ego_yaw  |
|Bytes 55-58 | ego_speed |
|Bytes 59-60 | horizontal_acceleration  |
|Bytes 61-62 | vertical_acceleration  |
|Bytes 63-64  | Number of satellites used for signal  |
|Bytes 65  | Fixed more status|
|Bytes 66 | CRC  |

<p>&nbsp;</p>