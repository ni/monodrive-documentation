# Batch Scenario Manager

monoDrive's Batch Generator Dashboard provides the client with visuals of custom configurations and result viewing for better testing. Everything about the simulator can be configured, from the material properties, to the vehicle properties, and to the sensor properties. The dashboard is designed to step through each configuration with pre-built and customizable properties. Once all configurations are made, deploy and run every combination of the properties selected in the simulation. Results show the status of the test after it has run, and keeps different batches organized for review. The Dashboard can be used as a local app on your computer or through a cloud deployment. The monoDrive Cloud Solution distributes batch processing to handle high volumes of AV testing jobs through a Kubernetes native application with easy deployment to any private or public cloud.

For viewing the full process on the local app see [Batch Scenario Dashboard Tutorial](https://www.youtube.com/watch?v=U1x_GU60LPg).


## Configurable Selections

### Dashboard Menu

The dashboard menu shows a summary of all the configurations that have been selected in the configuration selection process: vehicles and sensor configurations, scenario configurations, and weather configurations. On the bottom right corner of the dashboard menu, an user may use "Select Configurations" to make and/or edit selections.

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_starting_menu.png"/>
  </div>


### Vehicles & Sensors

When using closed loop simulation mode, the ego vehicle will be defined in the scenario file with the tag, `ego`. When making vehicle selections, select or add the vehicle that will be used in the closed loop scenario file. For example, this part of the `scenario_aebs.json` file in the UUT Python Example under [Configuring UUT](/scenario-dashboard/#configuring-unit-under-test).

```json
{
  "trigger_boxes": [],
  "vehicles": [
    {
      "body_color": "Carpaint_DarkGrey",
      "class_path": "/Game/Vehicles/sedan_monoDrive_02.sedan_monoDrive_02_C",
      "tags": [
        "vehicle",
        "dynamic",
        "car",
        "ego"
      ],
      ...
```

When using replay simulation mode, users can select multiple vehicle and sensor configurations to run. 

Select a vehicle configuration from monoDrive's vehicle models or add a custom vehicle as the ego vehicle in the simulation. The custom vehicle requires the directory path to the vehicle asset and enables users to change the color of the vehicle. By default all monoDrive's vehicle models spawn with white paint; if an user would like to customize the color on a monoDrive vehicle asset, select "Add New Vehicle" and use the path to the monoDrive vehicle asset and change it's color.

<div class="img_container" >
  <div style="border: 4px solid black">
  <video width=650px height=480px muted autoplay loop>
    <source src="https://cdn.monodrive.io/readthedocs/dashboard_vehicle_selection.mp4" type="video/mp4">
  </video>
  </div>
</div> 

For each vehicle selected, a user is given an option to create a new sensor configuration through the sensor editor tool.

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_sensors.png"/>
  </div>

#### Sensor Editor

The Sensor editor tool enables the user to import a JSON sensor configuration file or build a sensor configuration by adding and moving sensors on the vehicle model. Users are able to alter the position and location for every sensors. Many sensors offer further alterations depending on the sensor type such as image height, image width, and field of view. The "Import Config or Vehicle" Button enables users to import a vehicle model and also can import a sensor json file for easy uploading. After making all selections, the user needs to "Save Config" in the top left corner to save the configuration.

<div class="img_container">
  <video width=650px height=480px muted autoplay loop>
    <source src="https://cdn.monodrive.io/readthedocs/dashboard_sensor_editor.mp4" type="video/mp4">
  </video>
</div> 

### Unit Under Test (UUT) & Scenarios

 In the Local monoDrive Batch Scenario Manager Application, users will run and configure a Unit Under Test. The Unit Under Test will be configured for closed loop or replay mode. monoDrive has many pre-built trajectory files to run using the "Straight 5K" Map.

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_scenarios.png"/>
  </div>

For closed loop mode, the user can add as many scenario JSON files to run in the batch. 

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_closed_loop.png"/>
  </div>

In the Cloud version, users will be able to direct the application to the file directory of configured UUT files to run the simulation.

For details on writing a Unit Under Test and/or work with examples, see [Configuring UUT](/scenario-dashboard/#configuring-unit-under-test).

### Weather

The monoDrive Batch Scenario Manager offers many pre-build weather profiles and an option for customizing a new weather profile. The weather configuration provides customizable changes such as the angle of the sun, the amount of rain on the roads, amount of wind, wind angle, and rain fall in a scene. 

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_weather.png"/>
  </div>

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_custom_weather.png"/>
  </div>

## Viewing Results

After all selections are made, the batch scenario manager will either send the combinations to the simulator running locally through the Local Application or through a Cloud Deployment which will run a job process. The Dashboard will keep track of all batch runs for viewing past tests, and the Results will show tests that have passed and failed. Inside each individual test, the user can find more detailed information for further testing.

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

### Local Batch Generator Dashboard

  1. Download monoDrive Batch Generator Dashboard. *For access [contact us](https://www.monodrive.io/contact).*

  1. Extract all files from the downloaded monoDrive Dashboard file.

    <div class="img_container">
      <img class='lg_img' src="../imgs/dashboard_extract.jpeg"/>
    </div>

  1. On Windows, run monodrive.dashboard.exe. On Linux, run monodrive-dashboard.
 
  1. Chose or create a directory to save configuration files and results from the Unit Under Test and the Batch Generator Dashboard. In the Unit Under Test Example provided, the user would send the same asset directory as set in the Batch Directory.

    ``` python closed_loop_aebs.py --md_assets C:\Users\developer\Documents\BatchExample --md_loop ```

    Here is the same directory used upon opening the monoDrive Batch Generator Dashboard.

    <div class="img_container">
      <img class='wide_img' src="../imgs/dashboard_directory.png"/>
    </div>

  1. Open the Simulator or Scenario Editor.

    <div class="img_container">
      <img class='wide_img' src="../imgs/dashboard_simulator.png"/>
    </div>
  
  1. Open the Map used in the Unit Under Test and play the simulator.

    <div class="img_container">
      <video width=650px height=480px muted autoplay loop>
      <source src="https://cdn.monodrive.io/readthedocs/dashboard_set_map.mp4" type="video/mp4">
      </video>
    </div> 
 
  1. Run a [Unit Under Test](/scenario-dashboard/#configuring-unit-under-test).
 
  1. Make configuration selections in the monoDrive Scenario Batch Manager.

  1. Run the Batch by selecting "Deploy and Run" on the Dashboard then confirming or making any needed edits to the simulator.json file.

    - The batch will set a "READY" status to the Unit Under Test, which will run one of the configuration combinations made in the dashboard.

    - Once the test has completed or failed, it will send a status back to the dashboard and will save the result and update the results page for that Batch Run.

    - Users can replay any tests or view the individual results data for information on the test.


## Configuring Unit Under Test
  
  For an example of a UUT example, check out [Python Client UUT Examples](https://github.com/monoDriveIO/monodrive-python-client/tree/master/examples).

 <p>&nbsp;</p>