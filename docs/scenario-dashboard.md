# Batch Scenario Manager

monoDrive's Batch Scenario Manager allows for easy batch processing of AV testing jobs.
The application provides visuals of customizable configurations for running a scenario. 
Once all configurations are made, every combination of the properties selected for a scenario will run
as a test in the simulator. After tests are done running, users will be able to view the status of 
the results, and the manager will keep different batches organized for later review. 

For viewing the full process, see 
[Batch Scenario Dashboard Tutorial](https://www.youtube.com/watch?v=U1x_GU60LPg).

## Requirements

 - [monoDrive Simulator or Scenario Editor v.1.11](https://www.monodrive.io/register)
    - [Windows Setup](../monoDrive_home/getting_started/Windows)
    - [Linux Setup](../monoDrive_home/getting_started/Linux)
 - [Python Client](../python_client/quick_start)
 - monoDrive Batch Scenario Manager. 
    - For access [contact us](https://www.monodrive.io/contact).
    - Download monoDrive Batch Scenario Manager
    - Extract all files from the downloaded monoDrive Dashboard file.
    
      <div class="img_container">
      <img class='lg_img' src="../imgs/dashboard_extract.jpeg"/>
      </div>

## Running

  1. Inside the downloaded folder for monoDrive Dashboard, double-click or open 
  monodrive.dashboard.exe on Windows or monodrive-dashboard on Linux.

  1. When running for the first time, choose or create a directory to save configuration files 
  and results from the Unit Under Test and the Batch Scenario Manager.

    <div class="img_container">
      <img class='wide_img' src="../imgs/dashboard_directory.png"/>
    </div>
 
  1. Make [configuration selections](../scenario-dashboard/#configurable-selections) 
  in the monoDrive Scenario Batch Manager.

  1. Run the Batch by selecting "Deploy and Run" on the Dashboard then confirming or making any 
  needed edits to the simulator.json file.

    - The batch will set a "READY" status for the Unit Under Test to begin once running.

  1. Open the Simulator or Scenario Editor.

    <div class="img_container">
      <img class='wide_img' src="../imgs/dashboard_simulator.png"/>
    </div>

  1. Play or run the Simulator in Unreal Engine.
 
  1. Run a [Unit Under Test](../scenario-dashboard/#connecting-your-code). 
  In the Unit Under Test Example provided, the user would use the same asset directory 
  as set in the Batch Scenario Manager. For example, 

    ``` python closed_loop_aebs.py --md_assets C:\Users\developer\Documents\BatchExample --md_loop ```

  1. When the test is finished it will send the status of the test to the Batch Scenario Manager 
  to be saved and viewed in results. 
  See more about [viewing results](../scenario-dashboard/#viewing-results).


## Connecting your code
  
When connecting your code to run as a Unit Under Test (UUT), you will use the `jobs` module of 
the monoDrive client. This module handles all of the batch processing mechanics including job 
assignment, configuration, results storing, etc. It has been designed to allow for flexible 
execution across development, local batch processing, and cloud deployment.


#### Entry point
The main entry point is the `run_job` function. You will call this method as a wrapper around your
own main function. Using this wrapper allows for continuous looped processing (when needed)
and failure handling.

```python
from monodrive.jobs import run_job

def main():
    """main uut driver function"""
    # ... your code here ...

if __name__ == '__main__':
    run_job(main, verbose=True)
```

#### Configuration
To retrieve the configuration for the current job assignment, you will call the `get_simulator`
function. Individual config file paths will be parsed from the CLI arguments and pre-loaded into
the returned `Simulator` object.

```python
from monodrive.jobs import get_simulator

def main():
    """main uut driver function"""
    # ...
    simulator = get_simulator()
    # ...
```

#### Results  
Finally, at the end of main script, you will create a `Result` object and store it with the
`set_result` function. These metrics are defined by the user as desired. They will be surfaced
to the Batch Scenario Manager results page, but are not required for the batch processing to run 
properly.

```python
from monodrive.jobs import set_result, Result, ResultMetric

def main():
    """main uut driver function"""

    # ...

    result = Result()
    result.pass_result = True
    result.metrics.append(
        ResultMetric(
            name='max_lane_deviation',
            score=0.123
        )
    )
    set_result(result)
```
 
#### Example
For a full working UUT, please check out this 
[collision avoidance](https://github.com/monoDriveIO/monodrive-python-client/tree/master/examples/collision_avoidance)
use case, written in Python. This directory includes both a replay and closed loop mode example.

*(C++ example coming soon)*

## Configurable Selections

To get started with making configuration selections, follow instructions in 
[running](../scenario-dashboard/#running).

### Batch Scenario Manager Menu

The Batch Scenario Manager menu shows a summary of all the configurations that have been 
selected in the configuration selection process: vehicles and sensor configurations, scenario 
configurations, and weather configurations. On the bottom right corner of the menu, a user may 
use "Select Configurations" to make and/or edit selections.

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_starting_menu.png"/>
  </div>


### Vehicles & Sensors

Users can select multiple vehicle and sensor configurations to run. Select a vehicle configuration 
from monoDrive's vehicle models or add a custom vehicle as the ego vehicle for the simulation. 
The custom vehicle requires the directory path to the vehicle asset and enables users to change the 
color of the vehicle. By default all monoDrive's vehicle models spawn with white paint; if a user 
would like to customize the color on a monoDrive vehicle asset, select "Add New Vehicle" and use 
the path to the monoDrive vehicle asset and change it's color.

<div class="img_container" >
  <video width=650px height=480px muted autoplay loop>
    <source src="https://cdn.monodrive.io/readthedocs/dashboard_vehicle_selection.mp4" type="video/mp4">
  </video>
</div> 

For each vehicle selected, a user is given an option to create a new sensor configuration through 
the sensor editor tool.

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_sensors.png"/>
  </div>

#### Sensor Editor

The Sensor editor tool enables the user to import a JSON sensor configuration file or build a sensor 
configuration by adding and moving sensors on the vehicle model. Users are able to alter the 
position and location for every sensors. Many sensors offer further alterations depending on the 
sensor type such as image height, image width, and field of view. The "Import Config or Vehicle" 
Button enables users to import a vehicle model and also can import a sensor json file for easy 
uploading. After making all selections, the user needs to "Save Config" in the top left corner to 
save the configuration.

<div class="img_container">
  <video width=650px height=480px muted autoplay loop>
    <source src="https://cdn.monodrive.io/readthedocs/dashboard_sensor_editor.mp4" type="video/mp4">
  </video>
</div> 

### Unit Under Test (UUT) & Scenarios

 In the monoDrive Batch Scenario Manager, users will be able to run and configure a Unit Under Test. 
 The Unit Under Test will be configured for closed loop or replay mode. monoDrive has many pre-built 
 trajectory files to run using the "Straight 5K" Map.

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_scenarios.png"/>
  </div>

For closed loop mode, the user can add as many scenario JSON files to run in the batch. 

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_closed_loop.png"/>
  </div>

For details on writing a Unit Under Test and/or work with examples, see 
[Configuring UUT](../scenario-dashboard/#connecting-your-code).

### Weather

The monoDrive Batch Scenario Manager offers many pre-build weather profiles and an option for 
customizing a new weather profile. The weather configuration provides customizable changes such as 
the angle of the sun, the amount of rain on the roads, amount of wind, wind angle, and rain fall 
in a scene. 

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_weather.png"/>
  </div>

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_custom_weather.png"/>
  </div>

## Viewing Results

After all selections are made, the Batch Scenario Manager will keep track of all batch runs for 
viewing past tests, and the Results will show tests that have passed and failed. Inside each 
individual test, the user can find more detailed information or replay a test for further testing.

  <div class="img_container">
    <img class='wide_img' src="../imgs/dashboard_results.png"/>
  </div>

## Cloud Deployment

The Batch Scenario Manager can be used as a local application on your computer or through a 
cloud deployment. The local application enables users to view the test running in the Simulator, 
while the monoDrive Cloud Solution distributes batch processing to handle high volumes of AV testing 
jobs. The Cloud Solution is handled through a Kubernetes native application with easy deployment 
to any private or public cloud.

For more information, [contact us](https://www.monodrive.io/contact).

 <p>&nbsp;</p>