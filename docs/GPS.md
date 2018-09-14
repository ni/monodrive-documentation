<p align="center">
<img src="https://github.com/monoDriveIO/Client/raw/master/WikiPhotos/gpssensor.PNG" width="400" height="400" />
</p>

The configuration for a GPS sensor.

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

## Output Data
See [base sensor](Base-Sensor.md) for examples on how to get the sensor. All data that comes from sensor queues is a dictionary.

`data_gps = gps.get_message()`

### Parsed GPS Data Dictionary Keys and Values.

- **time_stamp (int):** Timestamp representing milliseconds since Sunday.
- **game_time (float):** Current game time of simulator, this value will be more prominent.
- **lat(float):** Latitude of Ego vehicle.
- **lng(float):** Longitude of Ego vehicle.

### Raw Output Data Format Parsed in `GPS.digest_frame()`

Following the data format found [here](https://github.com/swift-nav/libsbp/blob/master/docs/sbp.pdf).

- **Byte 0-1:** Start Byte
- **Bytes 1-3:** MSG_POS_LLH
- **Bytes 3-5:** Hash of the sensor id.
- **Byte 5-6:** Length of the payload.
- **Bytes 6-14:** Latitude bits represented as a double in double precision IEEE-754 format.
- **Bytes 14-22:** Longitude bits represented as a double in double precision IEEE-754 format.
- **Bytes 22-30:** Elevation bits represented as a double in double precision IEEE-754 format.
- **Bytes 30-32:** Horizontal accuracy, in millimeters.
- **Bytes 32-34:** Vertical accuracy, in millimeters.
- **Byte 34-35:** Number of satellites used for signal.
- **Byte 35-36:** Fixed more status.
- **Byte 36-37:** CRC.