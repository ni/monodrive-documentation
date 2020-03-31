## Configuration Examples

### Simulator Configuration
Configures the material properties for the elements in the simulation such as the road, concrete, steel ,etc. Also configures the maximum number of cars on the road and the rate at which they are spawn. 

```
{
 "id": "simulator_test",
 "server_ip": "127.0.0.1",
 "server_port": 8999,
 "client_ip": "127.0.0.1",
 "simulation_mode": 0,
 "phys_materials": {
   "Aluminum": {
     "specular_exponent": 15.0,
     "specular_coefficient": 0.95,
     "diffuse_coefficient": 0.26,
     "dielectric_constant": 10.0,
     "roughness": 0.15
   },
...
 },
 "traffic_configuration": {
   "max_vehicles": 40,
   "spawn_rate": 0.25
 },
 "client_settings": {
   "map": {
     "gis_anchor": { "x": 0, "y": 0, "z": 0},
     "point_delta": 100.0
   }
 }
}
```
<p>&nbsp;</p>



### Sensor Configuration

<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__camerac.png"/>
</p>

To change the configuration of any sensor, double-click on the sensor subVI and make the changes you need. 
Make sure you save the configuration as default value so that is persistent the next time you open your application.

<div class="img_container">
    <img class='lg_img' src="https://github.com/monoDriveIO/documentation/raw/links_and_giffs/docs/LV_client/quick_start_img/sensor_file.png"/>
</div>

   **NOTE:**
   Make sure the “listen_port” chosen is not used by another process or sensor.

#### Camera

<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__camerac.png"/>
</p>

```
{
    "type": "Camera",
   "id": "0",
   "packet_size": 23552,
   "listen_port": 8081,
   "display_process": false,
   "sensor_process": false,
   "location": {
     "x": -800.0,
     "y": 0.0,
     "z": 400.0
   },
   "rotation": {
     "pitch": -15.0,
     "yaw": 0.0,
     "roll": 0.0
   },
   "max_distance": 50000.0,
   "horizontal_fov_angle": 50.0,
   "fps": 1.0,
   "stream_dimensions": {
     "x": 768.0,
     "y": 768.0
   },
   "semantic_processing": false,
   "hdmi_streaming": false
}
```

#### GPS

<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__gpsc.png"/>
</p>

```
{
 "type": "GPS",
 "id": "5",
 "packet_size": 1472,
 "listen_port": 8090,
 "display_process": true,
 "sensor_process": true,
 "synchronized_display": true,
 "location": {
   "x": -75.0,
   "y": -25.0,
   "z": 245.0
 },
 "rotation": {
   "pitch": 0.0,
   "yaw": 0.0,
   "roll": 0.0
 },
 "fps": 1.0
}

```

#### Lidar

<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__lidarc.png"/>
</p>

```
{
 "type": "Lidar",
 "id": "0",
 "packet_size": 1472,
 "listen_port": 8093,
 "display_process": true,
 "sensor_process": true,
 "synchronized_display": true,
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
 "vertical_fov_angle": 30.0,
 "horizontal_resolution": 0.4,
 "fps": 1.0,
 "n_lasers": 32,
 "reset_angle": 0.0
}
```

#### Radar

<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__radarc.png"/>
</p>

```
 {
 "type": "Radar",
 "id": "2",
 "packet_size": 64000,
 "listen_port": 8092,
 "display_process": true,
 "sensor_process": true,
 "synchronized_display": true,
 "location": {
   "x": 250.0,
   "y": 0.0,
   "z": 50.0
 },
 "rotation": {
   "pitch": 0.0,
   "yaw": 0.0,
   "roll": 0.0
 },
 "num_samples_per_sweep": 1100,
 "fs": 150000000,
 "fc": 77000000000.0,
 "num_sweeps": 64,
 "range_max": 150.0,
 "sweep_num_for_range_max": 5.5,
 "range_resolution": 1.0,
 "max_velocity": 100.0,
 "max_targets": 100,
 "fps": 1.0,
 "elements": 8,
 "transmitter": {
   "peak_power": 5.0,
   "aperture": 0.000859,
   "gain": 13.5
 },
 "receiver": {
   "aperture": 0.000798,
   "nf": 10.0,
   "noise_temp": 290.0,
   "nb": 74000000.0,
   "gain": 20.0,
   "kb": 0.00059641065
 },
 "sbr": {
   "minimum_radar_distance": 5.0,
   "long_range_scan_distance": 150.0,
   "short_range_scan_distance": 60.0,
   "num_scans_azimuth": 20.0,
   "long_range_fov": 20.0,
   "short_range_fov": 90.0,
   "num_scans_elevation": 10.0,
   "elevation_fov": 5.0,
   "max_raycast_hits": 1
 }
}
```

#### RPM

<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__rpmc.png"/>
</p>

```
{
 "type": "RPM",
 "id": "0",
 "packet_size": 1472,
 "listen_port": 8086,
 "display_process": false,
 "sensor_process": false,
 "location": {
   "x": 0,
   "y": 0,
   "z": 0
 },
 "rotation": {
   "pitch": 0,
   "yaw": 0,
   "roll": 0
 },
 "fps": 1.0,
 "wheel_number": 0
}
```

#### IMU

<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__imuc.png"/>
</p>

```
{
 "type": "IMU",
 "id": "0",
 "packet_size": 1472,
 "listen_port": 8091,
 "display_process": true,
 "sensor_process": true,
 "synchronized_display": true,
 "location": {
   "x":-75.0,
   "y":-25.0,
   "z":245.0
 },
 "rotation": {
   "pitch":0.0,
   "yaw":0.0,
   "roll":0.0
 },
 "fps": 1.0
}
```

   **NOTE:**
   Make sure the “listen_port” chosen is not used by another process or sensor.


<p>&nbsp;</p>


### Sensor Output
Double-click on each sensor to look at the output of each sensor while running.

<div class="img_container">
    <img class='lg_img' src="https://github.com/monoDriveIO/documentation/raw/links_and_giffs/docs/LV_client/quick_start_img/output1.png"/>
</div>

<div class="img_container">
    <img class='lg_img' src="https://github.com/monoDriveIO/documentation/raw/links_and_giffs/docs/LV_client/quick_start_img/output2.png"/>
</div>

### Add Sensors

### Trajectory Configuration File

### Weather Configuration 

### Errors