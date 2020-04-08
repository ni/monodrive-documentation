# Weather

## monoDrive Dynamic Weather 

The monoDrive Simulator supports dynamic weather conditions that change the
visual and physical effects of the simulation. Pre-made weather profiles are
available in all of the monoDrive client versions which can be sent to 
the simulator to configure the current weather conditions. Additionally, the
monoDrive Scene Editor provides an interactive way of changing weather 
conditions and saving them to configuration files.

## Configuring Weather in monoDrive Scene Editor

In the monoDrive Scene Editor, the "Weather" actor can be found in any one of
the pre-made monoDrive Unreal Engine maps found in the `Content/Maps` directory.
The Weather actor can be added to custom maps easily by searching for the Actor 
in the "Modes" window as shown in the image below. Simply drag the actor into 
the custom map and it should now appear in the "World Outliner".

<p class="img_container">
  <img class="lg_img" src="../img/modes_weather_actor.png" />
</p>

To find the Weather Actor in the map, first go to the "World Outliner" window
and search for "weather" in the search box. The Weather Actor should appear as
shown in the image below. 

<p class="img_container">
  <img class="lg_img" src="../img/world_outliner_weather.png" />
</p>

Clicking on the Weather Actor in "World Outliner" should bring up the "Details"
window for the Actor. Scroll down to the "Weather Description" in the "Details"
window to find the weather settings pictured below.

<p class="img_container">
  <img class="lg_img" src="../img/details_weather_actor.png" />
</p>

* **Load File Button:** Loads the current weather profile from the Profile Name 
in file specified in the "File Name" field.
* **Save File Button:** Saves the current weather profile to the file specified 
in the "File Name" field. If the profile already existed in the file, it will be 
updated with the current values. When saved, both this `.ini` file and a `.json` 
file will be saved for use with the simulator and client.
* **Configuration File Name:** The name of the `.ini` file to save the current
profile to. 
* **Profile Name:** Located in the Name field under the Weather settings,
specfies the human readable name of the weather profile to save to.


### Weather File Formats

The monoDrive Weather settings are sent to the server from the client, but can
also be read in from a `.ini` file located in the 
`<VehicleAI Install Directory>\VehicleAI_Editor\Config\` directory. The default
configuration file for the simulator is `CarlaWeather.ini` and comes with
the shipped simulator. The `.ini` file should be used only with the  monoDrive
Simulator, clients should use the contents of the `.json` file to send to the
simulator during the initial connection.

After pressing the "Save File Button" pictured above, two files can be found
in the `Config` directory with the names specified in the 
"Configuration File Name" field. In the `.ini` file, the saved profile can 
be found under the specified "Profile Name" and appear as below:

```ini
[MyCoolNewProfile]
SunPolarAngle=44.586
SunAzimuthAngle=174
SunBrightness=0
SunDirectionalLightIntensity=20
SunDirectionalLightColor=(R=255.000000,G=255.000000,B=255.000000,A=1.000000)
SunIndirectLightIntensity=16.6
CloudOpacity=14.28
HorizontFalloff=20
ZenithColor=(R=0.034046,G=0.109247,B=0.295000,A=1.000000)
HorizonColor=(R=1.979559,G=2.586644,B=3.000000,A=1.000000)
CloudColor=(R=0.855778,G=0.919020,B=1.000000,A=1.000000)
OverallSkyColor=(R=1.000000,G=1.000000,B=1.000000,A=1.000000)
SkyLightIntensity=4
SkyLightColor=(R=247.000000,G=240.000000,B=225.000000,A=255.000000)
bPrecipitation=False
PrecipitationType=Rain
PrecipitationAmount=0
PrecipitationAccumulation=0
bWind=True
WindIntensity=10
WindAngle=0
bOverrideCameraPostProcessParameters=True
CameraPostProcessParameters.AutoExposureMethod=Histogram
CameraPostProcessParameters.AutoExposureMinBrightness=0.27
CameraPostProcessParameters.AutoExposureMaxBrightness=5
CameraPostProcessParameters.AutoExposureBias=-3.5
```

In the `.json` file, the weather profile can be found in the JSON group where 
the `id` field matches the "Profile Name":

```json
{
      "id": "Default",
      "SunPolarAngle": 44.586,
      "SunAzimuthAngle": 174,
      "SunBrightness": 50,
      "SunDirectionalLightIntensity": 15.092,
      "SunDirectionalLightColor": {
        "R": 255.0,
        "G": 239.0,
        "B": 194.0,
        "A": 1.0
      },
      "SunIndirectLightIntensity": 6,
      "CloudOpacity": 16.296,
      "HorizontFalloff": 3,
      "ZenithColor": {
        "R": 0.034046,
        "G": 0.109247,
        "B": 0.295,
        "A": 1.0
      },
      "HorizonColor": {
        "R": 0.659853, "G": 0.862215, "B": 1.0, "A": 1.0
      },
      "CloudColor": {
        "R": 0.855778, "G": 0.919005, "B": 1.0, "A": 1.0
      },
      "OverallSkyColor": {
        "R": 1.0, "G": 1.0, "B": 1.0, "A": 1.0
      },
      "SkyLightIntensity": 5.505,
      "SkyLightColor": {
        "R":0.149650, "G":0.161819, "B":0.205000, "A":0.000000
      },
      "bPrecipitation": false,
      "PrecipitationType": "Rain",
      "PrecipitationAmount": 0,
      "PrecipitationAccumulation": 0,
      "bWind": false,
      "WindIntensity": 20,
      "WindAngle": 0,
      "bOverrideCameraPostProcessParameters": true,
      "CameraPostProcessParameters": {
        "AutoExposureMethod": "Histogram",
        "AutoExposureMinBrightness": 0.27,
        "AutoExposureMaxBrightness": 5,
        "AutoExposureBias": -3.5
      }
}
```

## Using Weather in the LabView Client

* [Getting the weather configuration:](../LV_client/weather/mono__get__weatherc.md)
The LabView client stores all known weather profiles in this `.vi`. To add a new 
profile, paste in the contents of the profile `.json` file here. 
* [Sending weather information to the server:](../LV_client/weather/mono__weather__updatec.md) 
This `.vi` allows you to send the weather json from the `mono__get__weatherc.vi` 
to the server.
* [Determine the weather profile by index:](../LV_client/weather/mono__weather__idc.md)
This takes an index as input and produces the profile name of the weather as 
output.

## Using the Weather in the Python Client

TODO

## Using the Weather in the C++ Client

TODO

