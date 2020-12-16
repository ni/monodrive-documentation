# Scenario Creation

The monoDrive Scenario Editor allows users to create their own scenarios by 
controlling various different aspects of AI vehicle behavior. There are two types of files that can be created: the trajectory file for Replay Mode and the scenario file for Closed loop mode. 

<div class="img_container">
  <video width=650px height=480px muted autoplay loop>
    <source src="https://cdn.monodrive.io/readthedocs/scenario_editor_recording.mp4" type="video/mp4">
  </video>
</div> 

## Placing Vehicles in a Scene

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

## Detailed Vehicle Behavior Settings

The "Vehicle Settings" section of a vehicle's details contains controller settings
and behavior settings. The first section, "Vehicle Controller Settings", 
contains several settings relative to vehicle behavior for speed and control:

-   **Desired Speed:**  The desired speed of this vehicle in miles per hour. When Obeying Speed Limits of roads this is a delta above or below the speed limit.
	
	-   **Initial Speed:**  The initial speed of this vehicle in miles per hour. When Obeying Speed Limits of roads this is a delta above or below the speed limit
	
	-   **Delayed Started:**  The number of seconds to wait before moving this vehicle.
	- 
	-   **Show Vehicle Charts:**  If set, then vehicle dynamics charts will be shown
	
	-   **Debug Drawing:**  If set, then debug vehicle information will be drawn in the editor
	
	-   **Search Forward Distance:**  The number of centimeters to look ahead when computing AI actions. If speeds are much higher than 70mph consider increasing.
	
	- **Search Backward Distance:** The number of centimeters to look backward when computing AI actions
	
	- **Lane Offset**  The number of centimeters to drive from the lane center, positive is right, negative is left
	
	-   **Steer Forward Distance:**  The range of centimeters to look ahead when performing steering AI actions, interpolated based on `steerForwardSpeedRange`

	- **Steer Forward Distance During Lane Change:** The range of centimeters to look ahead when performing steering AI actions during a lane change, interpolated based on `steerForwardSpeedRange`

	- **Steer Forward Speed:** The range of speed (mph) used to LERP `steerForwardDistance`

	-   **Rolling stop speed:** The speed to travel before continuing after a legal stop
	
	-   **Distance to Stop Sign:**  The number of centimeters to stop before a controlled intersection

	-  **Spawn Offset** The offset to apply to the vehicle position prior to spawning

	- ** Acceleration Constraints** The minimum and maximum acceleration of the vehicle used by AutoPilot. cm/s^2
 
	- **Changing Lanes** Whether the vehicle is in the middle of a lane change
 
	- **Turn Pref** At the next turn the vehicle will prefer to take this turn. This will reset to NONE after the following turn

	- **Simple Speed Look Ahead** The number of centimeters to look ahead when performing simple speed control 
	
	- **Point PID** PID Controller settings for point control

	- **Speed PID** PID Controller settings for speed control

The "Vehicle Behavior Settings" section of a vehicle's settings contains options for 
obeying (or not) traffic laws:

-   **Ignores Lane Change Boxes:**  If set, all lane change trigger boxes will be ignored

-   **Ignore Other Vehicles:**  If set, all other vehicles will be ignored when computing AI actions

-   **Ignore Traffic Signals:**  If set, this vehicle will ignore all traffic control devices

-   **Obey Road Speed Limits:** If set, this vehicle will obey the speed limits of the road instead of its desired speed

-   **Limit Speed in Curves:**  If set, this vehicle will attempt to slow down in order to properly corner in turns

- **Initial Speed is Delta** If set, when vehicle is obeying speed limits, the initial velocity will be used as a delta above or below the speed limit. If vehicle is not obeying road speed limits this setting is ignored

-   **Destroy on Collision:**  If set, this vehicle will despawn after colliding with another physics body

-   **Snap to Lane:**  If set, this vehicle will search for the nearest lane and snap to it at spawn

- **Trigger Sleep Time** A vehicle might hit a dynamic trigger box more than once as they are in motion, to avoid double triggering, behavior can only be modified by the same trigger component again after this many seconds

## Tagging System

The Simulator's tagging system is used to specific attributes for desired actors in a recording. The tags for each actor can be seen by clicking on the actor and scrolling down to the actor's "Actor" group and looking under the "Tags" array. 

  <div class="img_container">
    <img class='lg_img' src="../imgs/vehicle_actor_tags.png"/>
  </div>

There are two categories of tags:

* **Desired Tags:** These are tags of elements that will be included in the trajectory.

* **Undesired Tags:** These are tags of elements that will not be included in the trajectory. Typically users do not want to record elements that have the "static" tag.

**NOTE:** You must have a vehicle with the tag "ego," otherwise it cannot be used with the monoDrive client.

* [LabVIEW Client Examples](../../LV_client/quick_start/LabVIEW_run_examples)
* [Python Client Examples](../../python_client/examples)
* [C++ Client Replay Modes](../../cpp_client/cpp_examples)

