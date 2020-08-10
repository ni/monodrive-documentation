# Batch Scenario Manager

monoDrive's Batch Generator Dashboard provides the client with visuals of custom configurations and result viewing for better testing. Everything about the simulator can be configured, from the material properties, to the vehicle properties, and to the sensor properties. The dashboard is designed to step through each configuration with pre-built and customizable properties. Once all configurations are made, deploy and run every combination of the properties selected in the simulation. Results show the status of the test after it has run, and keeps different batches organized for review. The Dashboard can be used as a local app on your computer or through a cloud deployment. The monoDrive Cloud Solution distributes batch processing to handle high volumes of AV testing jobs through a Kubernetes native application with easy deployment to any private or public cloud.

For viewing the full process on the local app see [Batch Scenario Dashboard Tutorial](https://www.youtube.com/watch?v=U1x_GU60LPg).


## Configurable Selections

### Vehicles & Sensors

Select a vehicle from monoDrive's vehicle models. For each vehicle selected, an user is provided with a default "all sensors" configuration or an option to create a new sensor configuration through the sensor editor tool.

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_vehicles.png"/>
  </div>

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_sensors.png"/>
  </div>

#### Sensor Editor

The Sensor editor tool enables the user to import a JSON sensor configuration file or build a sensor configuration by adding and moving sensors on the vehicle model. Users are able to alter the position and location for every sensors. Many sensors offer further alterations depending on the sensor type such as image height, image width, and field of view.

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_sensor_editor.png"/>
  </div>

### UUT & Scenarios

In the Local monoDrive Batch Scenario Manager Application, users will run and configure an UUT to use when running tests. In the Cloud version, users will be able to direct the application to the file directory of configured UUT files to run the simulation or use one of the prebuilt UUTs. There are many different scenarios to choose from in both Replay and Closed loop Modes. 

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_scenarios.png"/>
  </div>

### Weather

The monoDrive Batch Scenario Manager offers many pre-build weather profiles and an option for customizing a new weather profile. The weather configuration provides customizable changes such as the angle of the sun, the amount of rain on the roads, amount of wind, wind angle, and rain fall in a scene. 

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_weather.png"/>
  </div>

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_custom_weather.png"/>
  </div>

## Viewing Results

After all selections are made, the batch scenario manager will either send the permutations to the simulator running locally through the Local Application or through a Cloud Deployment which will run a job process. The Dashboard will keep track of all batch runs for viewing past tests, and the Results will show tests that have passed and failed. Inside each individual test, the user can find more detailed information for further testing.

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_results.png"/>
  </div>

## Requirements

 - [monoDrive Scenario Editor v.1.11](https://www.monodrive.io/register)
    - [Windows Setup](../monoDrive_home/getting_started/Windows)
    - [Linux Setup](../monoDrive_home/getting_started/Linux)
 - [Python Client](../python_client/quick_start)
 - Python for UUT

## Running

  1. Download monoDrive Batch Generator Dashboard. *For access [contact us](https://www.monodrive.io/contact).*

  1. Extract all files

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_extract.jpeg"/>
  </div>

  1. On Windows, run monodrive.dashboard.exe. On Linux, run monodrive-dashboard.
 
  1. Save results from the Unit Under Test (UUT) in the same directory that user specifies in app setup. 

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_store.jpeg"/>
  </div>

  1. Run simulator with the UUT configured map 
 
  1. Run UUT *See below for configuring UUT*
 
  1. Make selections in the monoDrive Scenario Batch Manager and run batch. 

  1. Refresh results and/or view later to see results after tests have run. 

## Configuring UUT
  
  For an example of a UUT example, check out [Python Client UUT Examples](https://github.com/monoDriveIO/monodrive-python-client/tree/master/examples)

 <p>&nbsp;</p>