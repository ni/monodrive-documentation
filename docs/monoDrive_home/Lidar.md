# LiDAR

The configuration for a LiDAR sensor is modeled after the Velodyne LiDAR. Currently we only support **16 and 32 laser**.

## LiDar

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
<p>&nbsp;</p>


## Configuration

### Configuration Tags
The location of the sensor can be modified in the "x", "y" and "z" axis with respect to the car.   
The sensor's orientation can be modified in the "yaw", "pitch" and "roll" axis.

- **max_distance**: The maximum distance, in centimeters, the LiDAR laser will travel.
- **horizontal_resolution**: The horizontal angle, in degrees, the LiDAR must rotate before shooting out the next set of lasers. Usually from 0.1 to 0.8.
- **rpms**: The expected number of revolutions per minute for a full 360 degree sweep. Controls the expected time between laser lines.
- **n_lasers**: The number of lasers the LiDAR sensor shoots out per sweep. This can be set to 16 (VLP-16), 32 (HDL-32).
- **reset_angle**: The angle that indicates a full revolution (i.e. full 360 degree revolution will start at this reported angle).
<p>&nbsp;</p>


### Raw Output Data Format
Each data packet from the LiDAR sensor contains **1206 bytes**.  
There are 12 data blocks, each block is **100 bytes** and there are **4 bytes** for the timestamp and **2 bytes** called factory bytes. The number of packets for a revolution depends on the **horizontal revolution**.  

- [VLP-16 Manual Download](http://velodynelidar.com/vlp-16.html)
- [HDL-32E Manual Download](http://velodynelidar.com/hdl-32e.html)

<p>&nbsp;</p>


## Visualize LiDAR output
1. Download [VeloView](https://www.paraview.org/VeloView/). 
2. Run VeloView
3. Click `Sensor Stream`, and select the correct configuration (Puck Hi-Res or HDL-32). 
3. Run the simulator and client normally when the stream data starts coming through VeloView will populate with the LiDAR data.

<p class="img_container">
  <img class="lg_img" src="https://github.com/monoDriveIO/Client/raw/master/WikiPhotos/lidarsensor.PNG"/>
</p>  
