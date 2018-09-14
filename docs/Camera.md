The configuration for a camera sensor that streams camera data and renders it into a view created by the client.

- `"semantic_processing": false`
<p align="center">
<img src="https://github.com/monoDriveIO/Client/raw/master/WikiPhotos/camerasensor.PNG" width="400" height="400" />
</p>

- `"semantic_processing": true`
<p align="center">
<img src="https://github.com/monoDriveIO/Client/raw/master/WikiPhotos/semanticcamerasensor.PNG" width="400" height="400" />
</p>

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
      "max_distance": float,
      "horizontal_fov_angle": float,
      "fps": int,
      "stream_dimensions": {
        "x": float,
        "y": float
      },
      "semantic_processing": bool,
      "hdmi_streaming": bool
}
```

- **hdmi_streaming**: A boolean representing whether the camera data gets streamed the HDMI, or the default of UDP.
- **stream_dimensions**: The dimensions of which the stream will stream over UDP. These dimensions are set in a dictionary with keys x and y with float values.
- **max_distance**: The max distance, in centimeters, the simulator's camera will look forward in the world.
- **horizontal_fov_angle**: The horizontal field of view angle, in degrees, of the simulator's camera.
- **semantic_processing**: A boolean that determines whether the camera will act as a semantic camera or a normal camera.

## Output Data
See [base sensor](Base-Sensor.md) for examples on how to get the queue that maintains the sensor's data. All data that comes from sensor queues is a dictionary.

data_camera= camera_q.get()

### Parsed Camera Data Dictionary Keys and Values.

- **time_stamp (int):** Timestamp representing milliseconds since Sunday.
- **game_time (float):** Current game time of simulator, this value will be more prominent.
- **image (List<List<float<float>>>):** A 3D list that represents a single image following the format of **stream_dimensions_x** x **stream_dimensions_y** x 4