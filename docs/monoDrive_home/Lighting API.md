# Lighting API 

</div>

<p class="img_container">
  <img class="half_screen_img" src="../img/lights.png" />
</p>

</div>

The monoDrive lights API allows users to create, configure and control lights in a simulation. A light configuration can be attached to an actor in the simulation and then controlled during the simulation by sending update commands. 

The API provides two commands: the lights configuration command `VehicleLightsConfigCommand_ID` which is used to set up and configure the lights at the beginning of the simulation, and the lights update command `VehicleLightsUpdateCommand_ID` which is used to update the state of the lights (intensity, location, orientation, etc.) during the simulation.

Lights are composed of one or more `URectangleLightObjects` or `UConeLightObjects` grouped together into a `ULightArray`, which is attached to an actor in the simulation. `UConeLightObjects` are composed of a `UPointLightComponent` which optionally provides backlighting in encslosures and a `USpotLightComponent` which provides the projected conical light. `URectangleLightObjects` use `RectLightComponents` instead to create a retangular light projection useful for LEDs.

Each backlight and directional light component are independetly moveable and controllable to create the correct projected and backlighting that models a vehicle's headlamps.

## Light Editor UI

<p class="img_container">
  <img class="lg_img" src="../img/light_array.png" />
</p>

The Light Editor UI allows the user to easily add, remove and configure conical and rectangular lights to a light array on an actor. The configuration can further be exported or imported to/from json for use with the client. This makes it very easy to visualize light arrays from the editor and verify it setup correctly before working with it from a control algorithm on the client.  

The best way to get started with the lights API is to create a vehicle, click Add Component as seen in the figure, add a light array component, and then add and manipulate cone and rectangular lights to the array.

**Export Light Array Button**: Opens a file dialogue to export the current light array to a `.json` file. This json file can then be used in the client or the light array created can be copied to a larger `.json` file that contains more arrays for configuration.  
**Import Light Array Button**: Opens a file dialogue to import a `.json` file overwriting the current array.  
**Cone Light Dropdown**: Lets the user select a ConeLightObject for deletion or copy.  
**Add Cone Light**: Adds a cone light to the array which is a copy of the currently selected cone light or a new light with defaults settings if no cone light exists or is selected.  
**Delete Cone Light**: Deletes the currently selected cone light from the list. Do not try to delete cone lights directly from the Component List or the ConeLights Array as this will not perform proper cleanup.  
**Rectangular Light Dropdown**: Same as Cone Light Dropdown but for the rectangular light array.  
**Add Rect Light**: Adds a rectangule light to the array which is a copy of the currently selected rectangule light or a new light with defaults settings if no rectangle light exists or is selected.  
**Delete Rect Light**: Deletes the currently selected rectangle light from the list. Do not try to delete rectangle lights directly from the Component List or the RectangleLights Array as this will not perform proper cleanup.  
**Id**: This is the name or id of the array which is needed for the client to address the id it wises to update or configure at runtime.  
**Cone Lights and Rectangle Lights**: These lists should not be directly modified. To modify a cone or rectangle light object instead select them in the Component List which resides above the Search Details search bar.  


## Configuration
The `VehicleLightsConfigCommand_ID` command configures a set of LED arrays and sets the initial state of each LED in the array. The file consists of an `actor_id` which specifies which actor in the scenario the lights will be attached to, the location and an array of arrays which can be placed anywhere on the actor and controlled at run time.

```json
{
	"actor_id": "",
	"lights": [{
		"array_id": "RightHeadLamp",
		"coneLights": [{
			"attach_socket": "",
			"attenuation_radius": 10000.0,
			"backlight_attenuation_radius": 20.0,
			"backlight_color": {
				"a": 1,
				"b": 1,
				"g": 1,
				"r": 1
			},
			"backlight_indirect_lighting_intensity": 10.0,
			"backlight_intensity": 20.0,
			"backlight_location": {
				"x": 0.0,
				"y": 0.0,
				"z": 0.0
			},
			"backlight_soft_source_radius": 2.0,
			"backlight_source_length": 1.0,
			"backlight_source_radius": 1.0,
			"backlight_temperature": 8000.0,
			"backlight_volumetric_scattering_intensity": 10.0,
			"color": {
				"a": 1,
				"b": 1,
				"g": 1,
				"r": 1
			},
			"description": "",
			"enable_backlight": true,
			"enable_directional_light": true,
			"enable_streaming": true,
			"ies_profile": "None",
			"indirect_lighting_intensity": 1.0,
			"inner_cone_angle": 8.0,
			"intensity": 1200.0,
			"led": 0,
			"listen_port": 0,
			"location": {
				"x": 0.0,
				"y": 0.0,
				"z": 0.0
			},
			"outer_cone_angle": 16.0,
			"parent_component_attachment": "",
			"raytrace_global_ilum": false,
			"raytrace_reflection": false,
			"raytrace_shadow": false,
			"receive_stream": true,
			"rotation": {
				"pitch": 0.0,
				"roll": 0.0,
				"yaw": 0.0
			},
			"soft_source_radius": 0.0,
			"source_length": 1.0,
			"source_radius": 0.0,
			"temperature": 8000.0,
			"type": "None",
			"use_attach_socket": false,
			"use_parent_component_attachment": false,
			"volumetric_scattering_intensity": 5.0,
			"wait_for_fresh_frame": true
		}],
		"location": {
			"x": 0.0,
			"y": 0.0,
			"z": 0.0
		},
		"rectLights": [{
			"attach_socket": "",
			"attenuation_radius": 10000.0,
			"backlight_attenuation_radius": 20.0,
			"backlight_color": {
				"a": 1,
				"b": 1,
				"g": 1,
				"r": 1
			},
			"backlight_indirect_lighting_intensity": 10.0,
			"backlight_intensity": 20.0,
			"backlight_location": {
				"x": 0.0,
				"y": 0.0,
				"z": 0.0
			},
			"backlight_soft_source_radius": 2.0,
			"backlight_source_length": 1.0,
			"backlight_source_radius": 1.0,
			"backlight_temperature": 8000.0,
			"backlight_volumetric_scattering_intensity": 10.0,
			"barn_door_angle": 30.0,
			"barn_door_length": 20.0,
			"color": {
				"a": 1,
				"b": 1,
				"g": 1,
				"r": 1
			},
			"description": "",
			"enable_backlight": true,
			"enable_directional_light": true,
			"enable_streaming": true,
			"ies_profile": "None",
			"indirect_lighting_intensity": 1.0,
			"intensity": 1200.0,
			"led": 0,
			"listen_port": 0,
			"location": {
				"x": 0.0,
				"y": 0.0,
				"z": 0.0
			},
			"parent_component_attachment": "",
			"raytrace_global_ilum": false,
			"raytrace_reflection": false,
			"raytrace_shadow": false,
			"receive_stream": true,
			"rotation": {
				"pitch": 0.0,
				"roll": 0.0,
				"yaw": 0.0
			},
			"source_height": 10.0,
			"source_width": 10.0,
			"temperature": 8000.0,
			"type": "None",
			"use_attach_socket": false,
			"use_parent_component_attachment": false,
			"volumetric_scattering_intensity": 5.0,
			"wait_for_fresh_frame": true
		}],
		"rotation": {
			"pitch": 0.0,
			"roll": 0.0,
			"yaw": 0.0
		}
	}]
}
```

## Light Parameters
**actor_id**: The actor these lights will be connected to.  
**array_id**: Identifier of the array, used by client to determine which array is being updated.  
**coneLights**: The array of ConeLightObjects which will be updated.  
**rectLights**: The array of RectangleLightObjects which will be updated.  
**location (at the array level)**: Relative location of the array with respect to the actor origin.
**rotation (at the array level)**: Relative rotation of the array with respect to the actor's coordinate frame.  
### Cone and Rectangle Lights (common parameters)
**attenuation_radius**: Radius (cm) used for attenuation of light of the directional light.  
**backlight_attenuation_radius**: Radius (cm) used for attenuation of light of the back light.  
**backlight_color**: Color of backlight, RGBA.  
**backlight_indirect_lighting_intensity**: Unused.
**backlight_intensity**: Light intensity in lumens of backlight.  
**backlight_location**: XYZ location relative to actors origin of the backlight.  
**backlight_soft_source_radius**: Radius in (cm) of the soft source, this light can bleed through surfaces.
**backlight_temperature**: The color temperature of the backlight.
**backlight_volumetric_scattering_intensity**: Unused.
**color**: Color of the directional light.
**enable_directional_light**: Disables the directional light, useful if only backlight is needed.  
**ies_profile**: The IES profile to use for the directional light if one is available, otherwise leave blank.  
**indirect_lighting_intensity**: Unused.  
**intensity**: Light intensity of the direcitonal light in lumens.  
**led**: The ID used to address this light.  
**location**: The location of the directional light.  
**raytrace_global_ilum**: Whether this light should be ray traced for global illumination.  Performance impact and will not be visible if the camera is not set to use ray tracing. Suggest off for low end machines.  
**raytrace_reflection**: Whether this light should be ray traced for reflections. Performance impact and will not be visible if the camera is not set to use ray tracing. Suggest off for low end machines.  
**raytrace_shadow**: Whether this light should be ray traced for shadows. Performance impact and will not be visible if the camera is not set to use ray tracing. Suggest off for low end machines.  
**rotation**: The relative rotation of the directional light with respect to the origin of the vehicle.  
**soft_source_radius**: Bleed through radius of the directional light.  
**temperature**: The light temperature for the directional light.  
**volumetric_scattering_intensity**: Unused.  

### Rectangle Light
**source_width**: The width of the rectangular light source element.  
**source_height**: The length of the rectangular light source element.  
**barn_door_angle**: The angle of the barn door which is the angle of the rectangular walls channeling the light.  
**barn_door_length**: The length of the barn door which will define how sharp the edges of the projected light are as longer is more focused and shorter is more diffuse.  

### Cone Light
**inner_cone_angle**: The inner angle of the cone. Light dropoff starts at this angle and decreases out to the outer cone angle.  
**outer_cone_angle**: The outer cone angle where the light will completely dropoff.  
**source_radius**: The radius at which the light source covers. This will reduce the realism of attenuation or IES. Defaults to 0.  
**soft_source_radius**: The source radius that is allowed to bleed through otherwise occluding objects, defaults to 0.  
**source_length**: The length size of the source of the light. Increasing stretches the light along the Z axis.  


The directional light component of a `ULightObject` can be configured using an [IES profile](https://docs.unrealengine.com/4.26/en-US/BuildingWorlds/LightingAndShadows/IESLightProfiles/). When specifying an IES profile,
the configuration will ignore the values of the other configuration parameters as all of those are part of the profile. The `ies_profile` parameter specifies the path, relative to the `Content` folder, for the
file. For example:
```
"ies_profile": "Materials/IES/JellyFish.uasset"
```

The simulator/scenario editor ships with several IES profiles which can be found inside the `Content/Materials/IES` folder. Additional IES profiles can be imported by following the instructions under 
[Importing and Assigning to Lights](https://docs.unrealengine.com/4.26/en-US/BuildingWorlds/LightingAndShadows/IESLightProfiles/#importingandassigningtolights) section or the UE4 IES Light Profile documentation page.


The monoDrive C++ client provides three structs for the configuration detailed above: LightsConfig, LEDArrayConfig, ConeLEDConfig, and RectLEDConfig. These are defined in the sensor_config.h header and provide conversion to/from json.

A sample configuration command using the C++ client code would look as follows:

```
LightsConfig lightsConfig; // the lights configuration
lightsConfig.actor_id = sim0.getEgoVehicleId();	// attach the configuration to the EGO vehicle

LEDArrayConfig lf_light_config; // the configuration for the left-front headlight
lf_light_config.array_id = "LeftFront";
lf_light_config.location = Location(90, -50, 75);

ConeLEDConfig led_config;
led_config.led = 0;
led_config.location = Location(0, 0, 0);
led_config.rotation = Rotation(0, -1, 0);
led_config.intensity = 1500;
led_config.backlight_intensity = 400;
led_config.attenuation_radius = 5000;
led_config.temperature = 8000;
led_config.backlight_temperature = 8000;
lf_light_config.coneLights.push_back(led_config);
lightsConfig.lights.push_back(lf_light_config);

LEDArrayConfig rf_light_config;	// do the same for the right-front headlight
rf_light_config.array_id = "RightFront";
rf_light_config.location = Location(90, 50, 75);
...
lightsConfig.lights.push_back(rf_light_config);

// send lights config command
sim0.sendCommand(ApiMessage(1002, VehicleLightsConfigCommand_ID, true, lightsConfig));
```


## Control
The `VehicleLightsUpdateCommand_ID` command adjusts settings for each LED in the specified LED array during the simulation, it updates every frame. The format for this command is similar to the configuration command except that it is not necessary to send the entire configuration. For example, if only the intensity values are being changed, then the command data would look as follows:

```json
{
	"actor_id": "ego_vehicle",	
	"lights": [
	{
		"array_id": "LeftFront", 
		"coneLights": [
        {
			"led": 0,
			"backlight_intensity": 400,
			"intensity": 1200
		},
	...
		]
},
...
	]
}
```

A sample update command using the C++ client code might look as follows:

```
while (simulationIsRunning)
{
    nlohmann::json update_config({
            { "actor_id", lightsConfig.actor_id },
            { "lights", nlohmann::json::array() }
        });
    for (int i = 0; i < lightsConfig.lights.size(); i++) {
        auto& lightArray = lightsConfig.lights[i];
        update_config["lights"].push_back({
                { "array_id", lightArray.array_id },
                { "coneLights", nlohmann::json::array() }
            });
        for (int j = 0; j < lightArray.coneLights.size(); j++) {
            update_config["lights"].back()["coneLights"].push_back({
                    { "led", j },
                    { "intensity", calculateIntensity(i, j) },
                    { "backlight_intensity", calculateBacklightIntensity(i, j) }
                });
        }
    }

    sim0.sendCommand(ApiMessage(1002, VehicleLightsUpdateCommand_ID, true, update_config));
    ...
}
```

## Car Light Positions

This table shows good initial position for setting ligths for each monoDrive vehicle.

| Vehicle Model | Left Side | Right Side|
| ------------ | ---------- | ---------- | 
| Compact | X=169.370102, Y=-63.970001, Z=80.844009 | X=169.370102, Y=63.970001, Z=80.844009 |
| Coupe | X=192.089249, Y=-77.371002, Z=70.172997 | X=192.089249, Y=77.371002, Z=70.172997 |
| Crossover | X=200.577194, Y=-79.397003, Z=83.918999 | X=200.577194, Y=79.397003, Z=83.918999 |
| Minivan | X=218.710449, Y=-75.148003, Z=87.083290 | X=218.710449, Y=75.148003, Z=87.083290 |
| Sedan 01 | X=187.382996, Y=-67.414001, Z=64.736000 | X=187.382996, Y=67.414001, Z=64.736000 |
| Sedan 02 | X=205.718002, Y=-71.880997, Z=73.824997 | X=205.718002, Y=71.880997, Z=73.824997 |
| SubCompact | X=95.000000, Y=-50.000000, Z=76.000000 | X=95.000000, Y=50.000000, Z=76.000000 |
| SUV | X=190.000000, Y=-62.905998, Z=80.236000 | X=190.000000, Y=62.905998, Z=80.236000 |
| Truck | X=252.341736, Y=-81.622002, Z=109.501999 | X=252.341736, Y=81.622002, Z=109.501999 |

