# LiDAR

The monoDrive LiDAR sensor simulates Velodyne LiDARs in both 16 and 32 laser 
variants. The output of the LiDAR matches the documented output of the 
commercial Velodyne LiDARs.

## Configuration

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

- **max_distance:** The maximum distance, in centimeters, the LiDAR laser will travel.
- **horizontal_resolution:** The horizontal angle, in degrees, the LiDAR must rotate before shooting out the next set of lasers. Usually from 0.1 to 0.8.
- **rpms:** The expected number of revolutions per minute for a full 360 degree sweep. Controls the expected time between laser lines.
- **n_lasers:** The number of lasers the LiDAR sensor shoots out per sweep. This can be set to 16 (VLP-16), 32 (HDL-32).
- **reset_angle:** The angle that indicates a full revolution (i.e. full 360 degree revolution will start at this reported angle).

## Raw Output

The output format of the **16 laser** LiDAR matches that of the 
[Velodyne Puck Hi-Res](https://velodynelidar.com/products/puck-hi-res/) in 
single return mode. The output of the **32 laser** LiDAR matches that of the 
[Velodyne HDL-32E](https://velodynelidar.com/products/hdl-32e/) in single return 
mode. Both of these models have a well documented format in their respective 
product manuals.

Each data packet from the LiDAR sensor contains **1206 bytes**.  
There are 12 data blocks, each block is **100 bytes** and there are **4 bytes** 
for the timestamp and **2 bytes** called factory bytes. The number of packets 
for a revolution depends on the **horizontal revolution**.  

### Visualizing LiDAR Output

The output of the LiDAR can be streamed over UDP (enabled by default with the 
[monoDrive LabVIEW client](../../LV_client/quick_start/LabVIEW_client_quick_start)). 
To visualize the output you will need to:

1. Download and run [VeloView](https://www.paraview.org/VeloView/)
1. Click `Sensor Stream`, and select the correct configuration (Puck Hi-Res or HDL-32).
1. Run the monoDrive Simulator, configure the monoDrive LabVIEW client with a LiDAR
1. When the LiDAR data starts streaming, VeloView will display the LiDAR points

<p class="img_container">
  <img class="lg_img" src="https://github.com/monoDriveIO/Client/raw/master/WikiPhotos/lidarsensor.PNG"/>
</p>  
