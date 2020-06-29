# Trajectory File Creation 

By attaching the monoDrive State Sensor to a vehicle, users can save scenarios to monoDrive
trajectory files that can then be used in the Replay modes of the monoDrive Simulator and Scenario Editor. 

## Using the State Sensor

In order to save all vehicles in a single simulation and generate a Trajectory File for Relay mode, a monoDrive State Sensor should be placed on the ego vehicle (i.e. the vehicle the simulation will 
control during playback). To attach the state sensor:

1. Click on the vehicle to designate as the ego vehicle

1. In the vehicle's "Details" window, click the "Add Component" button at the top and search for "State Sensor"

    <div class="img_container">
      <img class='wide_img' src="../imgs/ego_vehicle_state_sensor.png"/>
    </div>

1. Select the State Sensor component from the list to attach it.

1. Click on the `StateSensor` component in the vehicle's hierarchy in side of the "Details" window to bring up the settings for the sensor.

1. Setup the tagging system for the vehicle (see "Tagging System" below). Ensure that the selected vehicle contains the "ego" tag or the playback of the trajectory will not work.

1. Go to the `StateSensor`'s Streamer group in the "Details" window and select "File Streamer Component" as the "Streamer Type."

1. Under the "Streamer" section expand the "File Settings" and type in the name of the file to save the trajectory. If there is a file with the same name, it will be overwritten with the new file. 

    <div class="img_container">
      <img class='lg_img' src="../imgs/state_sensor_streamer_type.png"/>
    </div>

1. On the Sampling Control select 20 hz rate to save the state sensor data.


    <div class="img_container">
      <img class='lg_img' src="../imgs/streamer.png"/>
    </div>

1. Hit "Play" at the top of the Scenario Editor, the file set in the previous step should now be populated with all the desired tags.


## Replaying Trajectory Files

The scene that was recorded can be played back using the "Replay" or "Replay 
Step" modes of the monoDrive Simulator. For more information on these simulation
modes see:

* [LabView Client Examples](../../LV_client/quick_start/LabVIEW_run_examples)
* [Python Client Examples](../../python_client/examples)
* [C++ Client Replay Modes](../../cpp_client/cpp_examples)