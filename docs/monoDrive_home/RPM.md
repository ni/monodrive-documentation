# RPM 

Provides wheel revolutions per minute (RPM) information for the ego vehicle.

## Configuration

``` json
{
    "type": "RPM",
    "listen_port": 8600,
    "location": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "rotation": {
        "pitch": 0.0,
        "roll": 0.0,
        "yaw": 0.0
    },
    "wheelNumber": 0
}
```

**wheelNumber:** The number of the wheel to sample RPM values for. 0, 1, 2, 3 represent the front-left, front-right, rear-left, rear-right wheels respectively.

## Raw Output

The total sensor output is 24 bytes, where the first 16 bytes correspond to the monoDrive sensor header and the remaining 8 provide RPM information for the wheel. The following table specifies the output format for the wheel binary data:


| Byte   | Description |
| ------------ | ------------ |
|Bytes 0-3 | 32-bit integer representing the wheel number for the RPM sensor. |
|Bytes 4-7 | 32-bit floating point representing the wheel speed in RPM for the specific RPM sensor.  |
