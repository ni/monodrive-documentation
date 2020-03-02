## Lidar Sensor

The configuration for a LiDAR sensor that is modeled after the Velodyne LiDAR line. To view the lidar data download [VeloView](https://www.paraview.org/VeloView/). Run VeloView, click `Sensor Stream`, and select the correct configuration (Puck Hi-Res or HDL-32). Then run the simulator and client normally when the stream data starts coming through VeloView will populate with the lidar data.

<p align="center">
<img src="https://github.com/monoDriveIO/Client/raw/master/WikiPhotos/lidarsensor.PNG" width="400" height="400" />
</p>

```
{
  "type": "Lidar",
  "listen_port": 8200,
  "location": {
    "x":-75.0,
    "y":-25.0,
    "z":350.0
  },
  "rotation": {
    "pitch":0.0,
    "yaw":0.0,
    "roll":0.0
  },
  "max_distance": 8000.0,
  "horizontal_resolution": 0.8,
  "rpms": 300.0,
  "n_lasers": 16,
  "reset_angle": 0.0
}
```

- **horizontal_resolution**: The horizontal angle, in degrees, the LiDAR must rotate before shooting out the next set of lasers.
- **n_lasers**: The number of lasers the LiDAR sensor shoots out per sweep. This can be set to 16 (VLP-16), 32 (HDL-32), or 64 (HDL-64).
- **max_distance**: The max distance, in centimeters, the LiDAR laser will travel.
- **reset_angle**: The angle that indicates a full revolution (i.e. full 360 degree revolution will start at this reported angle).
- **rpms**: The expected number of revolutions per minute for a full 360 degree sweep. Controls the expected time between laser lines.

### Raw Output Data Format

- [VLP-16 Manual Download](http://velodynelidar.com/vlp-16.html)
- [HDL-32E Manual Download](http://velodynelidar.com/hdl-32e.html)

