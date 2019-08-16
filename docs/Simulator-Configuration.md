### Example JSON
```
{
  "id": "simulator_test",
  "server_ip": "127.0.0.1",
  "server_port": 8998,
  "client_ip": "127.0.0.1",
  "ego_config": {
    "body": {
        "type": "/Game/Vehicles_Vol1/VehicleBlueprints/Sedan4DBP.Sedan4DBP_C",
        "color": [ 0, 0, 0, 255 ]
    },
    "camera": {
      "location": {
        "x": -1000.0,
        "y": 0.0,
        "z": 500.0 
      },
      "rotation": {
        "pitch": 0.0,
         "yaw": 0.0,
         "roll": 0.0
      }
    }
  }, 
  "phys_materials": {
    "Aluminum": {
      "specular_exponent": 92.0,
      "specular_coefficient": 0.95,
      "diffuse_coefficient": 0.06,
      "dielectric_constant": 10.0,
      "roughness": 0.15
    },
    "Asphalt": {
      "specular_exponent": 10.0,
      "specular_coefficient": 0.10,
      "diffuse_coefficient": 0.95,
      "dielectric_constant": 7.0,
      "roughness": 0.48
    }
  }
}
```

- Open `examples/test.py`
- In the main method of the file there is an initialization of `simulator_config`. You can change which json file is used here.
- Open the configuration you want to use in the configurations folder and you can change the `server_ip` (the ip address of the computer running the simulator), the `server_port` (the port for the simulator, should be 8998), and the `client_ip` (the ip address of the computer running the simulator). If both simulator and client are running on the same computer use localhost as the ip addresses (127.0.0.1).
- The optional `ego_config` specifies the model, color, and UE4 chase camera location for the vehicle. If `ego_config` is omitted from the file, then a random vehicle and color will be chosen. The `body`'s `type` specifier allows you to provide a path the model BluePrint for the Ego Vehicle and the `color` specifies the vehicle's color. The `camera` key allows you to specify the location and orientation of the chase camera that will be rendered in the simulator window.
- In this configuration you can also change the physical material properties for the specular exponent (10-100), specular reflection coefficient (0.0-1.0), diffuse reflection coefficient (0.0-1.0), dielectric constant (0.10-10,000+), and roughness (0.0-1.0).