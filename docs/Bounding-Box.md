<p align="center">
<img src="https://github.com/monoDriveIO/Client/raw/master/WikiPhotos/boundingboxsensor.PNG" width="400" height="400" />
</p>

The configuration for a bounding box sensor.

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
See [base sensor](Base-Sensor.md) for examples on how to get the queue that maintains the sensor's data. All data that comes from sensor queues is a dictionary.

data_bounding_box = bounding_box_q.get()

### Parsed Bounding Box Data Dictionary Keys and Values.

- **time_stamp (int):** Timestamp representing milliseconds since Sunday.
- **game_time (float):** Current game time of simulator, this value will be more prominent.
- **distances (List<float<float>>):** Distances of targets from the Ego vehicle location.
- **angles (List<float<float>>):** Angles of targets relative to Ego vehicle's forward direction
- **x_points (List<float<float>>):** X points of targets relative to Ego vehicle's location
- **y_points (List<float<float>>):** Y points of targets relative to Ego vehicle's location
- **x_bounds (List<float<float>>):** Bounds of targets in the x direction, target widths
- **y_bounds (List<float<float>>):** Bounds of targets in the y direction, target lengths
- **box_rotations(List<float<float>>):** Rotation of targets relative to the Ego vehicle's rotation.
- **velocities (List<float<float>>):** Velocities of targets relative to the Ego Vehicle's speed
- **radar_distances (List<float<float>>):** Distances of targets from the Ego vehicle location that are in the Radar's FOV.
- **radar_angles (List<float<float>>):** Angles of targets relative to Ego vehicle's forward direction that are in the Radar's FOV
- **radar_velocities (List<float<float>>):** Velocities of targets relative to the Ego Vehicle's speed that are in the Radar's FOV.

### Raw Output Data Format Parsed in `BoundingBox.digest_frame()`

- **Bytes 0-2:** Start Bytes
- **Bytes 2-6:** Number of targets.
- **0...NumberOfTargets as i:**
  - **Bytes 6+25i-10+25i:** Distance bits represented as a float in single precision IEEE-754 format.
  - **Bytes 10+25i-14+25i:** Origin rotator yaw bits represented as a float in single precision IEEE-754 format.
  - **Bytes 14+25i-18+25i:** Bounds extent x bits represented as a float in single precision IEEE-754 format.
  - **Bytes 18+25i-22+25i:** Bounds extent y bits represented as a float in single precision IEEE-754 format.
  - **Bytes 22+25i-26+25i:** Relative rotation yaw bits represented as a float in single precision IEEE-754 format.
  - **Bytes 26+25i-30+25i:** Relative speed bits represented as a float in single precision IEEE-754 format.
  - **Byte 30+25i-31+25i:** Bool bit whether or not target is in field of view of the radar.