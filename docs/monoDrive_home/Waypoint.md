## Waypoint Sensor

<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/Client/raw/master/WikiPhotos/waypointsensor.PNG" />
</p>

The configuration for a waypoint sensor.
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
      "point_delta": float,
      "total_points": int
}
```

- **point_delta**: Distance, in centimeters, between waypoints. 
- **total_points**: Total number of waypoints per lane


## Output Data
See [base sensor](Common.md) for examples on how to get the sensor. All data that comes from sensor queues is a dictionary.

`data_waypoint = waypoint.get_message()`

### Parsed Waypoint Data Dictionary Keys and Values.

- **time_stamp (int):** Timestamp representing milliseconds since Sunday.
- **game_time (float):** Current game time of simulator, this value will be more prominent.
- **points_by_lane (List<List<float<float>>>):** A list of (x, Y) coordinates for each lane that are on the current road.
- **current_lane (int):** Current lane of of ego vehicle, index to get current lane coordinates from __point_by_lane__.
- **forward_vector (List<float<float>>):** Forward vector of the ego vehicle.
- **speed (float):** Speed in meters per second of the ego vehicle.
- **world_location (List<float<float>>):** World location of the ego vehicle.
- **rot_z (float):** Z/Yaw rotation of the ego vehicle.

### Raw Output Data Format Parsed in `Waypoint.digest_frame()`

- **Byte 0-4:** World location x coordinate bits represented as a float in single precision IEEE-754 format.
- **Bytes 4-8:** World location y coordinate bits represented as a float in single precision IEEE-754 format.
- **Bytes 8-12:** Forward vector x component bits represented as a float in single precision IEEE-754 format.
- **Byte 12-16:** Forward vector y component bits represented as a float in single precision IEEE-754 format.
- **Bytes 16-20:** Forward vector z component bits represented as a float in single precision IEEE-754 format.
- **Bytes 20-24:** Speed bits represented as a float in single precision IEEE-754 format.
- **Bytes 24-28:** Rotation yaw bits represented as a float in single precision IEEE-754 format.
- **Bytes 28-32:** Lane number.
- **Bytes 32-36:** Total number of lanes.
- **0...NumberOfLanes as i:**
  - **Bytes 36+8i-40+8i:** Latitude bits represented as a float in single precision IEEE-754 format.
  - **Bytes 40+8i-44+8i:** Longitude bits represented as a float in single precision IEEE-754 format.