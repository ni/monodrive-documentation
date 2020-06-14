# Creating Scenarios

## monoDrive Scenario Creation

The monoDrive Scenario Editor allows user to create their own scenarios by 
controlling various different aspects of AI vehicle behavior. By attaching
the monoDrive State Sensor to a vehicle, users can save scenarios to monoDrive
trajectory files that can then be used in the Replay modes of the monoDrive
Simulator and Scenario Editor.

<div class="img_container">
  <video width=650px height=480px muted autoplay loop>
    <source src="http://cdn.monodrive.io/readthedocs/scenario_editor_recording.mp4" type="video/mp4">
  </video>
</div> 


### Placing Vehicles in a Scene

The first step in creating a scenario is setting up the monoDrive vehicles in 
the desired scene and applying driving properties to each. To place a vehicle:

1. In the "Content Browser" navigate to the folder `Content/Vehicles`

    <div class="img_container">
      <img class='wide_img' src="../imgs/content_browser_vehicles.png"/>
    </div>

1. Drag one of the vehicles into the scene by click on the vehicle's icon
and putting it on the desired lane of travel.

1. In the "Details" window, search for the "Vehicle Settings" group and set
the desired speeds of the vehicle (in miles per hour).

    <div class="img_container">
      <img class='lg_img' src="../imgs/vehicle_settings_details.png"/>
    </div>

1. In the "Details" window, search for the "Vehicle Behavior" group and set the
desired vehicle behaviors for traffic laws.

    <div class="img_container">
      <img class='lg_img' src="../imgs/vehicle_behavior_details.png"/>
    </div>

1. At the top of the Scenario Editor hit the "Play" button. The vehicle should 
begin to travel down the lane closest to its original placement. 

1. Repeat this with multiple vehicles until the scene is populated as desired.

#### Detailed Vehicle Behavior Settings

The "Vehicle Settings" section of a vehicle's details contains several settings
relative to vehicle speed and control:

* **Desired Speed:** This is the desired speed (in mph) the vehicle will attempt to achieve if it is not obeying the lane speed limits.
* **Initial Speed:** This is the initial speed (in mph) the vehicle will start at.
* **Delayed Started:** If greater than zero, the vehicle will wait this many seconds before starting to move.
* **Show Vehicle Charts:** If ticked, then detailed plots of the vehicle's dynamics will be displayed.
* **Debug Drawing:** If ticked, then detailed graphics of vehicle desired behavior will be displayed in the scene.
* **Search Forward Distance:** The distance (in cm) the vehicle will look ahead for other vehicles.
* **Steer Forward Distance:** The distance (in cm) the vehicle will look ahead for corners.
* **Rolling stop speed:** The speed (in mph), the vehicle will achieve when stopping for traffic laws.
* **Distance to Stop Sign:** The distance (in cm) the vehicle will stop at when approaching a traffic control sign.
* **Follower PID:** PID parameters of the vehicle when following another vehicle.
* **Speed Maintain PID:** PID parameters of the vehicle when maintaining a desired speed.

The "Vehicle Behavior" section of a vehicle's details contains settings for 
obeying (or not) traffic laws:

* **Ignores Lane Change Boxes:** If set, then all lane change trigger boxes will be ignored by this vehicle.
* **Ignore Other Vehicles:** If set, then the vehicle will not change behavior relative to collision with other vehicles.
* **Overtake Slower Vehicles:** If set, then this vehicle will attempt to go around another vehicle if it is traveling slower.
* **Obey Road Speed Limits:*: If set, then this vehicle will use the current lane's speed limit rather than the desired speed of the vehicle.
* **Limit Speed in Curves:** If set, then this vehicle will attempt to achieve a speed such that it can successfully navigate a turn.
* **Ignore Traffic Signals:** If set, then traffic lights will be ignored.
* **Randomize Turn Preference:** If set, then the vehicle will randomly select a direction of travel at intersections, otherwise it will travel straight if possible.
* **Start Safe:** If set, then the vehicle will attempt to spawn in a safe location (to avoid collisions)
* **Destroy on Collision:** If set, then the vehicle will be removed from the simulation if it collides with an object.
* **Snap to Lane:** If set, then the vehicle will travel down the closest lane when spawned.


### Using the State Sensor to Record

In order to save all vehicles in a single simulation, a monoDrive State Sensor
should be placed on the ego vehicle (i.e. the vehicle the simulation will 
control during playback). To attach the state sensor:

1. Click on the vehicle to designate as the ego vehicle

1. In the vehicle's "Details" window, click the "Add Component" button at the top and search for "State Sensor"

    <div class="img_container">
      <img class='wide_img' src="../imgs/ego_vehicle_state_sensor.png"/>
    </div>

1. Select the State Sensor component from the list to attach it

1. Click on the `StateSensor` component in the vehicle's hierarchy in side of the "Details" window to bring up the settings for the sensor.

1. Setup the tagging system for the vehicle (see "Tagging System" below). Ensure that the selected vehicle contains the "ego" tag or the playback of the trajectory will not work.

1. Go to the `StateSensor`'s Streamer group in the "Details" window and select "File Streamer Component" as the "Streamer Type"

1. On the Sampling Control select 20 hz to save the state sensor data at this rate

1. Under the "Streamer" section expand the "File Settings" and type in the name of the file to save the trajectory. If there is a file with the same name, it will be overwritten with the new file. 

    <div class="img_container">
      <img class='lg_img' src="../imgs/state_sensor_streamer_type.png"/>
    </div>

1. Hit "Play" at the top of the Scenario Editor, the file set in the previous step should now be populated with all the desired tags.

#### Tagging System

The monoDrive State Sensor uses the Simulator's tagging system to record the 
desired actors for a trajectory file. The tags for each actor can be seen by 
clicking on the actor and scrolling down to the actor's "Actor" group and
looking under the "Tags" array. 

  <div class="img_container">
    <img class='lg_img' src="../imgs/vehicle_actor_tags.png"/>
  </div>

For the state sensor there are two categories of tags:

* **Undesired Tags:** These are tags of elements that will not be included in the trajectory. Typically users do not want to record elements that have the "static" tag.

**NOTE:** You must have a vehicle with the tag "ego" when recording a trajectory 
file or the trajectory cannot be replayed in the monoDrive Simulator.

### Replaying Trajectory Files

The scene that was recorded can be played back using the "Replay" or "Replay 
Step" modes of the monoDrive Simulator. For more information on these simulation
modes see:

* [LabView Client Examples](../../../LV_client/quick_start/LabVIEW_run_examples)
* [Python Client Examples](../../../python_client/examples)
* [C++ Client Replay Modes](../../../cpp_client/cpp_examples)