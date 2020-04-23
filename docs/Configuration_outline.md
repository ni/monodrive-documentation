# Title

## About

Description: Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.


Have image here: 
<p class="img_container">
  <img class="wide_img" src="" />
</p>
<p>&nbsp;</p>

## How/ Where it's used in the Simulation

Description: Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.

Images and information

## Configuring in the Simulator

To find....
<p>&nbsp;</p>

### CARLA Format

The monoDrive Weather can be easily imported from CARLA settings by putting 
the desired profile into the
    `<VehicleAI Install Directory>\VehicleAI_Editor\Config\CarlaWeather.ini` 
file.
This file is used to load the initial weather profiles into the simulator so
they can be edited in the weather settings. The contents of a profile in `.ini`
should look similar to the following:

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
<p>&nbsp;</p>

### Saving Weather Profiles

The default configuration file for the simulator is `CarlaWeather.ini` and comes 
with the shipped simulator. This file loads up several pre-determined weather 
profiles for editing in the Scenario Editor. These profiles can be edited in 
the Scenario Editor and saved out to a `.json` format that is usable in the
monoDrive clients.

After pressing the "Save File Button" pictured above, two files can be found
in the `Config` directory with the names specified in the 
"Configuration File Name" field. In the `.ini` file, the saved profile can 
be found under the specified "Profile Name" and appear as below:

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
<p>&nbsp;</p>

## Using Weather in the LabView Client
For the LabVIEW client, the weather profiles that are exported can be pasted 
into the JSON `profiles` array located in the `mono_get_weather.vi` VI. 

* [Getting the weather configuration:](../LV_client/weather/mono__get__weatherc.md)
The LabView client stores all known weather profiles in this `.vi`. To add a new 
profile, paste in the contents of the profile `.json` file here. 

After opening the VI's GUI: the weather profiles can be found on the left-hand
side of the interface. Simply copy and paste your custom weather profile into 
the array:

<p class="img_container">
  <img class="wide_img" src="../img/get_weather_vi.png" />
</p>
<p>&nbsp;</p>

## Using the Weather in the Python Client

TBD

## Using the Weather in the C++ Client

TBD

