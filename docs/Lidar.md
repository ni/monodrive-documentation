The configuration for a LiDAR sensor that is modeled after the Velodyne LiDAR line. To view the lidar data download [VeloView](https://www.paraview.org/VeloView/). Run VeloView, click `Sensor Stream`, and select the correct configuration (16, 32, or 64). Then run the simulator and client normally when the stream data starts coming through VeloView will populate with the lidar data.

<p align="center">
<img src="https://github.com/monoDriveIO/PythonClient/blob/master/WikiPhotos/lidarsensor.PNG" width="400" height="400" />
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
        "x": float
        "y": float,
        "z": float
      },
      "rotation": {
        "pitch": float,
        "yaw": float,
        "roll": float
      },
      "max_distance": float,
      "vertical_fov_angle": float,
      "horizontal_resolution": float,
      "fps": float,
      "n_lasers": float (16, 32, or 64)
}
```

- **max_distance**: The max distance, in centimeters, the LiDAR laser will travel.
- **vertical_fov_angle**: The vertical field of view angle, in degrees, for the LiDAR.
- **horizontal_resolution**: The horizontal angle, in degrees, the LiDAR must rotate before shooting out the next set of lasers.
- **n_lasers**: The number of lasers the LiDAR sensor shoots out per sweep. This can be set to 16 (VLP-16), 32 (HDL-32), or 64 (HDL-64).

### Raw Output Data Format

- [VLP-16 Manual Download](http://velodynelidar.com/vlp-16.html)
- [HDL-32E Manual Download](http://velodynelidar.com/hdl-32e.html)
- [HDL-64E Manual Download](http://velodynelidar.com/hdl-64e.html)

