# Viewport

The Viewport sensor allows a client to configure the camera that is displayed in 
the monoDrive Simulator or Scenario Editor. 

## Configuration

``` json
{
  "type": "ViewportCamera",
  "channel_depth": 1,
  "channels": "bgra",
  "dynamic_range": 50.0,
  "focal_length": 9.0,
  "fov": 60.0,
  "fstop": 1.39999997615814,
  "listen_port": 0,
  "location": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0
  },
  "max_distance": 50000.0,
  "max_shutter": 0.00139999995008111,
  "min_shutter": 0.000500000023748726,
  "rotation": {
      "pitch": 0.0,
      "roll": 0.0,
      "yaw": 0.0
  },
  "sensor_size": 9.06999969482422,
  "stream_dimensions": {
      "x": 512,
      "y": 512
  }
}

```

**fov:** Sets the horizontal angle of view of the viewport camera.

### Configuration Examples  

The image on the left is the camera output and on the right the image on the 
Viewport on the VehicleAI Editor.

<p class="img_container">
  <img class="wide_img" src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/configuration/viewport/comparison_camera_viewport.png" />
</p>
