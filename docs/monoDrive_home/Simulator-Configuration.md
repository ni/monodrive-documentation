# Simulator
 
## Simulator Configuration

``` json
{
   "client_ip": "127.0.0.1",
   "client_settings": {
      "gui": {
         "fps": 1.0
      },
      "logger": {
         "control": "info",
         "network": "info",
         "scenario": "info",
         "sensor": "debug",
         "simulator": "info",
         "vehicle": "info"
      },
      "map": {
         "gis_anchor": {
               "x": 0,
               "y": 0,
               "z": 0
         },
         "point_delta": 100.0
      }
   },
   "ego_config": {
      "body": {
         "color": "CarPaint_White.CarPaint_White",
         "type": "Blueprint'/Game/Vehicles/crossover_monoDrive_01.crossover_monoDrive_01_C'"
      },
      "vehicle_dynamics": "physx"
   },
   "id": "simulator_test",
   "map": "Almono",
   "phys_materials": {
      "Aluminum": {
         "dielectric_constant": 10.0,
         "diffuse_coefficient": 0.26,
         "roughness": 0.15,
         "specular_coefficient": 0.95,
         "specular_exponent": 15.0
      },
      "Asphalt": {
         "dielectric_constant": 7.0,
         "diffuse_coefficient": 0.65,
         "roughness": 0.65,
         "specular_coefficient": 0.03,
         "specular_exponent": 1.0
      },
      "Concrete": {
         "dielectric_constant": 7.0,
         "diffuse_coefficient": 0.65,
         "roughness": 0.0,
         "specular_coefficient": 0.1,
         "specular_exponent": 0.0
      },
      "Glass": {
         "dielectric_constant": 4.0,
         "diffuse_coefficient": 0.15,
         "roughness": 0.1,
         "specular_coefficient": 0.8,
         "specular_exponent": 80.0
      },
      "LandScape": {
         "dielectric_constant": 10.0,
         "diffuse_coefficient": 0.5,
         "roughness": 0.6,
         "specular_coefficient": 0.5,
         "specular_exponent": 10.0
      },
      "Plastic": {
         "dielectric_constant": 2.8,
         "diffuse_coefficient": 0.6,
         "roughness": 0.09,
         "specular_coefficient": 0.4,
         "specular_exponent": 25.0
      },
      "Road": {
         "dielectric_constant": 7.0,
         "diffuse_coefficient": 0.65,
         "roughness": 0.65,
         "specular_coefficient": 0.1,
         "specular_exponent": 1.0
      },
      "Steel": {
         "dielectric_constant": 0.0,
         "diffuse_coefficient": 0.33,
         "roughness": 0.23,
         "specular_coefficient": 1.0,
         "specular_exponent": 26.0
      },
      "Tire": {
         "dielectric_constant": 2.8,
         "diffuse_coefficient": 0.6,
         "roughness": 0.15,
         "specular_coefficient": 0.4,
         "specular_exponent": 25.0
      },
      "Tree": {
         "dielectric_constant": 10.0,
         "diffuse_coefficient": 0.97,
         "roughness": 0.6,
         "specular_coefficient": 0.05,
         "specular_exponent": 10.0
      },
      "Wheel": {
         "dielectric_constant": 10.0,
         "diffuse_coefficient": 0.06,
         "roughness": 0.15,
         "specular_coefficient": 0.95,
         "specular_exponent": 92.0
      }
   },
   "server_ip": "127.0.0.1",
   "server_port": 8999,
   "simulation_mode": 1,
   "traffic_configuration": {
      "max_vehicles": 10,
      "spawn_rate": 0.25
   }
}
```
<p>&nbsp;</p>

### Configuration Tags

- **client_ip**: The ip address of the computer running the simulator. If both simulator and client are running on the same computer use localhost as the ip address (127.0.0.1).
- **map**: The name of the map to load.
- **server_port**: the TCP port for the simulator, typically 8999.
- **simulation_mode**: Closed loop (0), replay(1) or hil(2).
- **ego_config**: Specifies the model and color of the ego vehicle. The `body`'s `type` specifier allows you to provide a path to the model BluePrint for the ego vehicle and the `color` specifies the vehicle's color. If `ego_config` is omitted from the file, then a random vehicle and color will be chosen. Only applies when the simulator is running on replay mode. For closed loop it will use the car specified in the scenario file.
- **traffic_configuration**: The behavior of other vehicles on simulation. 
  - **max_vehicles**: Maximum number of vehicles spawn in the map.
  - **min_desired_speed**: Specify the maximum amount the speed of any vehicle can be **reduced** from the speed limit of the road.  Units on km per hour.
  - **max_desired_speed**: Specify the maximum amount the speed of any vehicle can be **increased** from the speed limit of the road. Units on km per hour.
  - **swarm_traffic"**: Set to *True* if you want to control the distance where other vehicles can spawn as well as when they can disappear from the map.
  - **spawn_leash_distance**: If **swarm_traffic** enabled. Specify the maximum distance a car can be from the ego vehicle before it gets destroyed. Units on centimeters.
  - **min_spawn_range**: If **swarm_traffic** enabled. Specify the minimum distance where a car can be spawn from the ego vehicle. Units on centimeters.
  - **vertical_offset**: Specify the distance from the road a car is being spawn on the Z direction.
  - **overtake_slower_vehicles**: Set to *True* if you want to allow cars at a faster speed to overtake cars moving slower. 
- **phys_materials**: Specify the physical material properties for the specular exponent (10-100), specular reflection coefficient (0.0-1.0), diffuse reflection coefficient (0.0-1.0), dielectric constant (0.10-10,000+), and roughness (0.0-1.0).

<p>&nbsp;</p>
