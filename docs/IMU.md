## IMU Sensor

<p align="center">
<img src="https://github.com/monoDriveIO/Client/raw/master/WikiPhotos/imusensor.PNG" />
</p>

The configuration for an IMU sensor.

```
[
  {
    "type": "IMU",
    "listen_port": 8500,
    "location": {
      "x":-75.0,
      "y":-25.0,
      "z":245.0
    },
    "rotation": {
      "pitch":0.0,
      "yaw":0.0,
      "roll":0.0
    }
  }

]
```

## Output Data
See [base sensor](Base-Sensor.md) for examples on how to get the sensor. All data that comes from sensor queues is a dictionary.

`data_waypoint = imu_sensor.get_message()`

### Parsed IMU Data Dictionary Keys and Values.

- **time_stamp (int):** Timestamp representing milliseconds since Sunday.
- **game_time (float):** Current game time of simulator, this value will be more prominent.
- **acceleration_vector (List<float<float>>):** Acceleration vector of the Ego Vehicle.
- **angular_velocity_vector (List<float<float>>):** Angular velocity vector of the Ego Vehicle. 
- **timer (int):** Timer based on this [protocol](http://files.microstrain.com/dcp/Inertia-Link-3DM-GX2-data-communications-protocol.pdf).

### Raw Output Data Format Parsed in `IMU.digest_frame()`

Following the data format found [here](http://files.microstrain.com/dcp/Inertia-Link-3DM-GX2-data-communications-protocol.pdf).

- **Byte 0-1:** Start Byte
- **Bytes 1-5:** x acceleration bits represented as a float in single precision IEEE-754 format.
- **Bytes 5-9:** y acceleration bits represented as a float in single precision IEEE-754 format.
- **Byte 9-13:** z acceleration bits represented as a float in single precision IEEE-754 format.
- **Bytes 13-17:** x angular velocity bits represented as a float in single precision IEEE-754 format.
- **Bytes 17-21:** y angular velocity bits represented as a float in single precision IEEE-754 format.
- **Bytes 21-25:** z angular velocity bits represented as a float in single precision IEEE-754 format.
- **Bytes 25-29:** Timestamp as indicated in the above link.
- **Bytes 29-31:** Checksum.
- **Byte 31-35:** Time of week.