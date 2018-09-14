The configuration for a multi camera sensor that streams all of the cameras in the `camera_ids` array through a single UDP port into one rendered image.

<p align="center">
<img src="https://github.com/monoDriveIO/Client/raw/master/WikiPhotos/multicamerasensor.PNG" width="1000" height="250" />
</p>

```
{
      "type": string,
      "id": string,
      "packet_size": int,
      "listen_port": int,
      "fps": int,
      "display_process": bool,
      "sensor_process": bool,
      "hdmi_streaming": bool,
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
      "stream_dimensions": {
        "x": float,
        "y": float
      },
      "camera_ids": [
          strings
      ]
}
```

- **hdmi_streaming**: A boolean representing whether the camera data gets streamed the HDMI, or the default of UDP.
- **stream_dimensions**: The dimensions of which the stream will stream over UDP.
  - *x*: The x dimension.
  - *y*: The y dimension.
- **camera_ids**: An array of strings that representing the ids of the cameras that are to be streamed through the multi camera.

## Output Data
See [base sensor](Base-Sensor.md) for examples on how to get the queue that maintains the sensor's data. All data that comes from sensor queues is a dictionary.

data_camera= camera_q.get()

### Parsed Camera Data Dictionary Keys and Values.

- **time_stamp (int):** Timestamp representing milliseconds since Sunday.
- **game_time (float):** Current game time of simulator, this value will be more prominent.
- **image (List<List<float<float>>>):** A 3D list that represents a single image following the format of **stream_dimensions_x** x **stream_dimensions_y** x 4