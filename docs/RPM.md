## RPM Sensor

<p align="center">
<img src="https://github.com/monoDriveIO/Client/raw/master/WikiPhotos/rpmsensor.png" width="400" height="400" />
</p>

The configuration for an RPM sensor.

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

## Output Data
See [base sensor](Base-Sensor.md) for examples on how to get the sensor. All data that comes from sensor queues is a dictionary.

`data_rpm = rpm.get_message()`

### Parsed RPM Data Dictionary Keys and Values.

- **time_stamp (int):** Timestamp representing milliseconds since Sunday.
- **game_time (float):** Current game time of simulator, this value will be more prominent.
- **wheel_number(int):** Wheel number the sensor is attached to.
- **wheel_rpm(float):** Rotations per minute.


### Raw Output Data Format Parsed in `RPM.digest_frame()`

- **Bytes 0-4:** The wheel number for the RPM sensor.
- **Bytes 4-8:** The wheel speed in RPM for the specific RPM sensor.