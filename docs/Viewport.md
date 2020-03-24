## Viewport Sensor

The monoDrive Viewport sensor allows you to configure the information displayed on your screen.

### Configuration

```
[
{
  "type": "ViewportCamera",
  "listen_port": 0,
 "location": {
      "x": 0.0,
      "y": 0.0,
      "z": 250.0
    },
    "rotation": {
      "pitch": 0.0,
      "yaw": 0.0,
      "roll": 0.0
    },
    "stream_dimensions": {
      "x": 512.0,
      "y": 512.0
    },
    "dynamic_range": 50,
    "fov": 60.0,
    "focal_length": 9.0,
    "fstop": 1.4,
    "min_shutter": 0.000500,
    "max_shutter": 0.001400,
    "sensor_size": 9.07,
    "channels": "rgba"
}

]
```

## Ouput Examples

Image on the left is the camera output and on the right the image on the Viewport on the VehicleAI Editor.

<p align="center">
<img src="https://github.com/monoDriveIO/documentation/blob/update-docs/WikiPhotos/LV_client/sensors/configuration/viewport/comparison_camera_viewport.png" />
</p>
