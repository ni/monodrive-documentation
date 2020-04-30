# Viewport

The Viewport sensor allows configuration of the camera that is displayed in the 
monoDrive Simulator or Scenario Editor. 

## Configuration

```
[
  {
    "type": "ViewportCamera",
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
      "fov": 60.0,
  }
]
```

**fov:** Sets the horizontal angle of view of the viewport camera.

### Configuration Examples  

The image on the left is the camera output and on the right the image on the 
Viewport on the VehicleAI Editor.

<p class="img_container">
  <img class="wide_img" src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/configuration/viewport/comparison_camera_viewport.png" />
</p>
