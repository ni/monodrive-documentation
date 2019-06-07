## Camera Sensor

The configuration for a camera sensor that streams camera data and renders it into a view created by the client.

### Semantic vs. Normal Camera

There are two types of camera that have the same configuration. The `"type": "Camera"` will provide a a full
color camera stream that is post-processed for illumination:
<p align="center">
<img src="https://github.com/monoDriveIO/Client/raw/master/WikiPhotos/camerasensor.PNG" width="400" height="400" />
</p>

The `"type": "SemanticCamera"` provides a grayscale camera stream with optional bounding boxes for dynamic objects in the scene:
<p align="center">
<img src="https://github.com/monoDriveIO/Client/raw/master/WikiPhotos/semanticcamerasensor.PNG" width="400" height="400" />
</p>

```
{
    "type": "Camera",
    "id": "0",
    "packet_size": 23552,
    "listen_port": 8081,
    "display_process": false,
    "sensor_process": true,
    "location": {
        "x": -260.0,
        "y": -0.0,
        "z": 85.0
    },
    "rotation": {
        "pitch": -0.0,
        "yaw": 180.0,
        "roll": 0.0
    },
    "stream_dimensions": {
        "x": 1280.0,
        "y": 800.0
    },
    "max_distance": 50000.0,
    "dynamic_range": 50,
    "angle_of_view": 60.0,
    "focal_length": 3.0,
    "fstop": 1.4,
    "min_shutter": 0.0005,
    "max_shutter": 0.0014,
    "sensor_size": 12.8,
    "paint_boxes": true,
    "pixel_area_cull": 40
}
```

- **stream_dimensions**: The dimensions of which the stream will stream over TCP. These dimensions are set in a dictionary with keys x and y with float values.
- **max_distance**: The max distance, in centimeters, the simulator's camera will look forward in the world.
- **dynamic_range**: The dynamic range of the camera in decibels.
- **angle_of_view**: (optional) If set this will override the angle of view calculated fromt he sensor size and lens focal length.
- **focal_length**: The focal length of the lens in millimeters.
- **fstop**: The f-stop value for the lens in stops.
- **min_shutter**: The minimum shutter speed, in seconds, for dynamic exposure.
- **max_shutter**: The maximum shutter speed, in seconds, for dynamic exposure.
- **sensor_size**: The size of the camera sensor in millimeters.
- **paint_boxes**: (semantic camera) If set to true, then labeled semantic boxes will be drawn on the semantic image.
- **pixel_area_cull**: (semantic camera) The minimum number of pixels a target must be to have a semantic box drawn around it.

## Output Data
See [base sensor](Base-Sensor.md) for examples on how to get the sensor. All data that comes from sensor queues is a dictionary.

`data_camera= camera.get_message()`

### Parsed Camera Data Dictionary Keys and Values.

- **time_stamp (int):** Timestamp representing milliseconds since Sunday.
- **game_time (float):** Current game time of simulator, this value will be more prominent.
- **image (List<List<float<float>>>):** A 3D list that represents a single image following the format of **stream_dimensions_x** x **stream_dimensions_y** x 4
