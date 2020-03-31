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


### Sensor Configuration

<p class="img_container">
<img class="lg_img" src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/mono__camerac.png"/>
</p>

To change the configuration of any sensor, double-click on the sensor subVI and make the changes you need. 
Make sure you save the configuration as default value so that is persistent the next time you open your application.

<div class="img_container">
    <img class='wide_img' src="https://github.com/monoDriveIO/documentation/raw/links_and_giffs/docs/LV_client/quick_start_img/sensor_file.png"/>
</div>


### Sensor Output

### Add Sensors

### Trajectory Configuration File

### Weather Configuration 

### Errors