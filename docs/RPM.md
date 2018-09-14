## RPM Sensor

<p align="center">
<img src="https://github.com/monoDriveIO/Client/raw/master/WikiPhotos/rpmsensor.png" width="400" height="400" />
</p>

The configuration for an RPM sensor.

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
      "fps": float,
      "wheel_number": int
}
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