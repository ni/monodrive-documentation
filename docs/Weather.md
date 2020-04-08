# monoDrive Weather

The monoDrive Simulator Supports dynamic weather through a set of JSON messages 
that can be sent from the client to simulator.

## Base Weather Configuration

This is the base weather configuration.

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

## Example Weather Condition

This is an example weather condition message:

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

### Using Weather in the LabView Client

* [Link to monoDrive LabView Client Get Weather](LV_client/weather/mono__get__weatherc.md)
* [Link to monoDrive LabView Client ID](LV_client/weather/mono__weather__updatec.md)
* [Link to monoDrive LabView Client Update](LV_client/weather/mono__weather__idc.md)


