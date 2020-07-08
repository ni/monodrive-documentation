# Simulator
 
## Simulator Configuration

``` json
{
   "client_ip": "127.0.0.1",
   "ego_config": {
      "body": {
         "color": "CarPaint_White.CarPaint_White",
         "type": "Blueprint'/Game/Vehicles/crossover_monoDrive_01.crossover_monoDrive_01_C'"
      },
      "vehicle_dynamics": "physx"
   },
    "traffic_configuration":{	
      "max_vehicles":20,	
      "min_desired_speed":-20.0,	
      "max_desired_speed":10.0,	
      "swarm_traffic":true,	
      "spawn_leash_distance":15000,	
      "min_spawn_range":2000,	
      "vertical_offset":25,	
      "overtake_slower_vehicles":true	
   },
   "id": "simulator_test",
   "map": "Almono",
   "phys_materials": {
    "Aluminum": {
      "specular_exponent": 15.0,
      "specular_coefficient": 0.95,
      "diffuse_coefficient": 0.26,
      "dielectric_constant": 10.0,
      "roughness": 0.15
    },
    "Asphalt": {
      "specular_exponent": 1.0,
      "specular_coefficient": 0.03,
      "diffuse_coefficient": 0.65,
      "dielectric_constant": 7.0,
      "roughness": 0.65
    },
    "Concrete": {
      "specular_exponent": 0.0,
      "specular_coefficient": 0.1,
      "diffuse_coefficient": 0.65,
      "dielectric_constant": 7.0,
      "roughness": 0.0
    },
    "Glass": {
        "specular_exponent": 80.0,
        "specular_coefficient": 0.80,
        "diffuse_coefficient": 0.15,
        "dielectric_constant": 4.0,
        "roughness": 0.10
      },
    "Gravel": {
        "specular_exponent": 10.0,
        "specular_coefficient": 0.1,
        "diffuse_coefficient": 0.95,
        "dielectric_constant": 7.0,
        "roughness": 0.48
      },
    "LandScape": {
        "specular_exponent": 10.0,
        "specular_coefficient": 0.50,
        "diffuse_coefficient": 0.50,
        "dielectric_constant": 10.0,
        "roughness": 0.60
      },
    "Plastic": {
        "specular_exponent": 25.0,
        "specular_coefficient": 0.40,
        "diffuse_coefficient": 0.60,
        "dielectric_constant": 2.8,
        "roughness": 0.09
      },
    "ReflectiveMaterial": {
      "specular_exponent": 10.0,
      "specular_coefficient": 0.5,
      "diffuse_coefficient": 0.5,
      "dielectric_constant": 1.0,
      "roughness": 1.0
    },
    "Road": {
      "specular_exponent": 1.0,
      "specular_coefficient": 0.1,
      "diffuse_coefficient": 0.65,
      "dielectric_constant": 7.0,
      "roughness": 0.65
    },
    "RoadPaint": {
        "specular_exponent": 10.0,
        "specular_coefficient": 10.0,
        "diffuse_coefficient": 2.0,
        "dielectric_constant": 7.0,
        "roughness": 1.0
      },
    "Steel": {
      "specular_exponent": 26.0,
      "specular_coefficient": 1.0,
      "diffuse_coefficient": 0.33,
      "dielectric_constant": 0.0,
      "roughness": 0.23
    },
    "Tire": {
      "specular_exponent": 25.0,
      "specular_coefficient": 0.40,
      "diffuse_coefficient": 0.60,
      "dielectric_constant": 2.8,
      "roughness": 0.15
    },
    "Tree": {
      "specular_exponent": 10.0,
      "specular_coefficient": 0.05,
      "diffuse_coefficient": 0.97,
      "dielectric_constant": 10.0,
      "roughness": 0.60
    },
    "Wheel": {
      "specular_exponent": 92.0,
      "specular_coefficient": 0.95,
      "diffuse_coefficient": 0.06,
      "dielectric_constant": 10.0,
      "roughness": 0.15
    }
   },
   "server_ip": "127.0.0.1",
   "server_port": 8999,
   "simulation_mode": 1
}
```
<p>&nbsp;</p>

### Configuration Tags

- **client_ip**: The IP address of the computer running the client software. 
- **server_ip**: The IP address of the computer running the monoDrive simulator. 
- **ego_config**: Specifies the model and color of the ego vehicle. The `body`'s `type` specifier allows you to provide a path to the model BluePrint for the ego vehicle and the `color` specifies the vehicle's color. If `ego_config` is omitted from the file, then a random vehicle and color will be chosen. Only applies when the simulator is running on replay mode. For closed loop it will use the car specified in the scenario file.
- **id** The name for the simulator configuration i.e. "simulator with no traffic"
- **map**: The name of the map to load.
- **phys_materials**: Specify the physical material properties for the specular exponent (10-100), specular reflection coefficient (0.0-1.0), diffuse reflection coefficient (0.0-1.0), dielectric constant (0.10-10,000+), and roughness (0.0-1.0).
- **server_port**: the TCP port for the simulator, typically 8999.
- **simulation_mode**: Closed loop (0), replay(1) or hil(2).
- **traffic_configuration**: The behavior of other vehicles on simulation. 
  - **max_vehicles**: Maximum number of vehicles spawn in the map.
  - **min_desired_speed**: Specify the maximum amount the speed of any vehicle can be **reduced** from the speed limit of the road.  Units on km per hour.
  - **max_desired_speed**: Specify the maximum amount the speed of any vehicle can be **increased** from the speed limit of the road. Units on km per hour.
  - **swarm_traffic"**: Set to *True* if you want to control the distance where other vehicles can spawn as well as when they can disappear from the map.
  - **spawn_leash_distance**: If **swarm_traffic** enabled. Specify the maximum distance a car can be from the ego vehicle before it gets destroyed. Units on centimeters.
  - **min_spawn_range**: If **swarm_traffic** enabled. Specify the minimum distance where a car can be spawn from the ego vehicle. Units on centimeters.
  - **vertical_offset**: Specify the distance from the road a car is being spawn on the Z direction.
  - **overtake_slower_vehicles**: Set to *True* if you want to allow cars at a faster speed to overtake cars moving slower. 

<p>&nbsp;</p>
