# Simulator 

The monoDrive Simulator provides users with the ability to test AV algorithms in one of several pre-made monoDrive Simulator levels. The Open Source monoDrive clients provide an API and examples for connecting to the Simulator for simulating traffic scenarios and replaying monoDrive trajectory files. The clients allow users to configure and test any number of configurations of monoDrive high-fidelity sensors.

## Getting Started

1. Install Unreal Engine **4.23** from [here](https://www.unrealengine.com/en-US/).


1. In Epic Games Launcher, and select Marketplace >> Browse. Search for "VehicleSim Dynamics" and download this free plugin. 
    - Alternatively, from Epic Games website, sign-in and download the plugin [here](https://www.unrealengine.com/marketplace/en-US/product/carsim-vehicle-dynamics).


1. Ensure you have the latest NVIDIA drivers for your [CUDA-enabled](https://developer.nvidia.com/cuda-gpus) graphics card.

    <div class="img_container">
        <img class='lg_img' src="../easier_device_driver_update.png"/>
    </div>

    **NOTE**
    Even if not prompted by the system, a restart is recommended to ensure the new drivers are correctly detected prior to running any simulations.

    **NOTE**
    In case the driver cannot be updated using the above method, please visit http://www.nvidia.com/Download/index.aspx to download the [latest driver](https://www.nvidia.comnvidia.com).
    

1. Download the monoDrive Simulator from [here](https://www.monodrive.io/register).

<p>&nbsp;</p>
<!-- 
## Simulator Configuration

```
{
	"server_ip": "127.0.0.1",
	"map": "Almono",
	"server_port": 8999,
	"simulation_mode": 2,
	"ego_config": {
		"body": {
			"type": "/Game/Vehicles/crossover_monoDrive_01.crossover_monoDrive_01_C",
			"color": "Carpaint_White.Carpaint_White"
		}
	},
	"traffic_configuration": {
		"max_vehicles": 20,
		"min_desired_speed": -20.0,
		"max_desired_speed": 10.0,
		"swarm_traffic": true,
		"spawn_leash_distance": 15000,
		"min_spawn_range": 2000,
		"vertical_offset": 25,
		"overtake_slower_vehicles": true
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
	}
}
```
<p>&nbsp;</p>

### Configuration Tags

- **server_ip**: The ip address of the computer running the simulator. If both simulator and client are running on the same computer use localhost as the ip address (127.0.0.1).
- **map**: The name of the map to load.
- **server_port**: the TCP port for the simulator, typically 8999.
- **simulation_mode**: Closed loop (0), replay(1) or hil(2).
- **ego_config**: Specifies the model and color of the Ego vehicle. The `body`'s `type` specifier allows you to provide a path the model BluePrint for the Ego Vehicle and the `color` specifies the vehicle's color. If `ego_config` is omitted from the file, then a random vehicle and color will be chosen.
- **traffic_configuration**: The behavior of other vehicles on simulation. 
  - **max_vehicles**: Maximum number of vehicles spawn in the map.
  - **min_desired_speed**: Specify the maximum amount the speed of any vehicle can be **reduced** from the speed limit of the road.  Units on km per hour.
  - **max_desired_speed**: Specify the maximum amount the speed of any vehicle can be **increased** from the speed limit of the road. Units on km per hour.
  - **swarm_traffic"**: Set to *True* if you want to control the distance where other vehicles can spawn as well as when they can disappear from the map.
  - **spawn_leash_distance**: If **swarm_traffic** enabled. Specify the maximum distance a car can be from the Ego vehicle before it gets destroyed. Units on centimeters.
  - **min_spawn_range**: If **swarm_traffic** enabled. Specify the minimum distance where a car can be spawn from the Ego vehicle. Units on centimeters.
  - **vertical_offset**: Specify the distance from the road a car is being spawn on the Z direction.
  - **overtake_slower_vehicles**: Set to *True* if you want to allow cars at a faster speed to overtake cars moving slower. 
- **phys_materials**: Specify the physical material properties for the specular exponent (10-100), specular reflection coefficient (0.0-1.0), diffuse reflection coefficient (0.0-1.0), dielectric constant (0.10-10,000+), and roughness (0.0-1.0).

<p>&nbsp;</p> -->
