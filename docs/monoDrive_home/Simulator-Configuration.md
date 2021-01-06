# Simulator
 
## Simulator Configuration

``` json
{
  "id": "simulator",
  "server_ip": "127.0.0.1",
  "server_port": 8999,
  "traffic_configuration": {
    "max_vehicles": 0,
    "min_desired_speed": 0.0,
    "max_desired_speed": 1.0,
    "spawn_leash_distance": 15000,
    "min_spawn_range": 2000,
    "swarm_traffic": true,
    "vertical_offset": 5,
    "overtake_slower_vehicles" : false
  },
  "phys_materials": {
    
  },
  "simulation_mode": 2,
   "map": "Almono",
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
<p>&nbsp;</p>

### Configuration Tags

- **server_ip**: The IP address of the computer running the monoDrive simulator. 
- **id** The name for the simulator configuration i.e. "simulator with no traffic"
- **map**: The name of the map to load.
- **phys_materials**: Specify the physical material properties for the specular exponent (10-100), specular reflection coefficient (0.0-1.0), diffuse reflection coefficient (0.0-1.0), dielectric constant (0.10-10,000+), and roughness (0.0-1.0).
- **server_port**: the TCP port for the simulator, typically 8999.
- **simulation_mode**: Closed loop (0), replay(1), hil(2), closed loop fixed time step mode(3).
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
