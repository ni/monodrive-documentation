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
   "simulation_mode": 1,
   "traffic_configuration": {
      "max_vehicles": 10,
      "spawn_rate": 0.25
   }
}
```
<p>&nbsp;</p>

### Configuration Tags

- **client_ip**: The IP address of the computer running the client software. 
- **client_settings**
   - **gui**
   - **logger**
   - **map**
      - **gis_anchor**
      - **point_delta**
- **ego_config**: Specifies the model and color of the ego vehicle. The `body`'s `type` specifier allows you to provide a path to the model BluePrint for the ego vehicle and the `color` specifies the vehicle's color. If `ego_config` is omitted from the file, then a random vehicle and color will be chosen. Only applies when the simulator is running on replay mode. For closed loop it will use the car specified in the scenario file.

<!-- should mention something about vehicle dynamics- physx -->

- **id**
- **map**: The name of the map to load.

<!-- Just want to make sure the description for the phys materials is correct -->
- **phys_materials**: Specify the physical material properties for the specular exponent (10-100), specular reflection coefficient (0.0-1.0), diffuse reflection coefficient (0.0-1.0), dielectric constant (0.10-10,000+), and roughness (0.0-1.0).
- **server_ip**: The IP address of the computer running the monoDrive simulator.
- **server_port**: the TCP port for the simulator, typically 8999.
- **simulation_mode**: Closed loop (0), replay(1) or hil(2).
- **traffic_configuration**: The behavior of other vehicles on simulation. 
  - **max_vehicles**: Maximum number of vehicles spawn in the map.
  - **spawn_rate**:

<p>&nbsp;</p>
