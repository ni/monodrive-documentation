### Example JSON
```
{
  "id": "simulator",
  "server_ip": "127.0.0.1",
  "map": "Almono_P",
  "server_port": 8999,
  "client_ip": "127.0.0.1",
  "simulation_mode": 2,
  "ego_config": {
  "body": {
      "type": "/Game/Vehicles_Vol1/VehicleBlueprints/Sedan4DBP.Sedan4DBP_C",
      "color": [ 0, 0, 0, 255 ]
  }
},
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
  "traffic_configuration": {
    "max_vehicles": 50,
    "min_desired_speed": 0.0,
    "max_desired_speed": 5.0,
    "spawn_leash_distance": 15000,
    "min_spawn_range": 2000,
    "swarm_traffic": true,
    "vertical_offset": 10,
    "overtake_slower_vehicles" : false
  },
  "client_settings": {
    "map": {
      "gis_anchor": { "x": 0, "y": 0, "z": 0},
      "point_delta": 100.0
    },
    "gui":{
      "fps": 1.0
    },
    "logger": {
      "sensor": "debug",
      "network": "info",
      "control": "info",
      "scenario": "info",
      "simulator": "info",
      "vehicle": "info"
    }
  }
}
```

- **max_vehicles**: Maximum number of vehicles spawn in the map.
- Open the configuration you want to use in the configurations folder and you can change the `server_ip` (the ip address of the computer running the simulator), the `server_port` (the port for the simulator, should be 8998), and the `client_ip` (the ip address of the computer running the simulator). If both simulator and client are running on the same computer use localhost as the ip addresses (127.0.0.1).
- The optional `ego_config` specifies the model, color, and UE4 chase camera location for the vehicle. If `ego_config` is omitted from the file, then a random vehicle and color will be chosen. The `body`'s `type` specifier allows you to provide a path the model BluePrint for the Ego Vehicle and the `color` specifies the vehicle's color. The `camera` key allows you to specify the location and orientation of the chase camera that will be rendered in the simulator window.
- In this configuration you can also change the physical material properties for the specular exponent (10-100), specular reflection coefficient (0.0-1.0), diffuse reflection coefficient (0.0-1.0), dielectric constant (0.10-10,000+), and roughness (0.0-1.0).