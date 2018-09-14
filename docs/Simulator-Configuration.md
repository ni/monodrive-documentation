### Example JSON
```
{
  "id": "simulator_test",
  "server_ip": "127.0.0.1",
  "server_port": 8998,
  "client_ip": "127.0.0.1",
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
- In this configuration you can also change the physical material properties for the specular exponent (10-100), specular reflection coefficient (0.0-1.0), diffuse reflection coefficient (0.0-1.0), dielectric constant (0.10-10,000+), and roughness (0.0-1.0).