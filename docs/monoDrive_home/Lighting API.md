# Lighting API 

The monoDrive lights API allows users to create, configure and control lights in a simulation. A light configuration can be attached to an actor in the simulation and then controlled during the simulation by sending update commands. 

The API provides two commands: the lights configuration command `VehicleLightsConfigCommand_ID` which is used to set up and configure the lights at the beginning of the simulation, and the lights update command `VehicleLightsUpdateCommand_ID` which is used to update the state of the lights (intensity, location, orientation, etc.) during the simulation.

Lights are composed of one or more `ULightObjects` grouped together into a `ULightArray`, which is attached to an actor in the simulation. `ULightObjects` are composed of a `UPointLightComponent` and a `USpotLightComponent`.


## Configuration
The `VehicleLightsConfigCommand_ID` command configures a set of LED arrays and sets the initial state of each LED in the array:

```json
{
  "actor_id"	// the id of the actor to attach the lights to
  "lights": [
    {
      "array_id" // id of this light array, used for control
      "position" // x, y, z relative to actor center
      "rotation"// yaw, pitch, roll relative to actor center
      "lights": [
        // array of led configuration
        {
          "led": 0, 	// led index in array
          "position": ,	// x,y,z relative to array
          "rotation":	,// yaw, pitch, roll relative to array
          // backlight (UPointLightComponent) settings
          "backlight_intensity"
          "backlight_color"
          "backlight_attenuation_radius"
          "backlight_source_radius"
          "backlight_soft_source_radius"
          "backlight_source_length"
          "backlight_temperature"
          "backlight_indirect_lighting_intensity"
          "backlight_volumetric_scattering_intensity"

          // spotlight (USpotLightComponent) settings
          "inner_cone_angle"
          "outer_cone_angle"
          "intensity"
          "color"
          "attenuation_radius"
          "temperature"
          "source_radius"
          "soft_source_radius"
          "source_length"
          "indirect_lighting_intensity"
          "volumetric_scattering_intensity"
          "ies_profile"
        },
        ...
      ]
    },
    ...
  ]
}
```

The monoDrive C++ client provides three structs for the configuration detailed above: LightsConfig, LEDArrayConfig and LEDConfig. These are defined in the sensor_config.h header and provide conversion to/from json.

A sample configuration command using the C++ client code would look as follows:

```

LightsConfig lightsConfig; // the lights configuration
lightsConfig.actor_id = sim0.getEgoVehicleId();	// attach the configuration to the EGO vehicle

LEDArrayConfig lf_light_config; // the configuration for the left-front headlight
lf_light_config.array_id = "LeftFront";
lf_light_config.location = Location(90, -50, 75);

LEDConfig led_config;
led_config.led = 0;
led_config.location = Location(0, 0, 0);
led_config.rotation = Rotation(0, -1, 0);
led_config.intensity = 1500;
led_config.backlight_intensity = 400;
led_config.attenuation_radius = 5000;
led_config.temperature = 8000;
led_config.backlight_temperature = 8000;
lf_light_config.lights.push_back(led_config);
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
      "lights": [
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
                { "lights", nlohmann::json::array() }
            });
        for (int j = 0; j < lightArray.lights.size(); j++) {
            update_config["lights"].back()["lights"].push_back({
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


## Example

</div>

<p class="img_container">
  <img class="half_screen_img" src="../img/lights.png" />
</p>

</div>

