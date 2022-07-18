# Scenario File Creation

The monoDrive Scenario Editor allows users to create their own scenarios by 
controlling various different aspects of AI vehicle behavior. The scenario file is used when running the Simulator and/or the Scenario Editor in closed loop mode.

<div class="img_container">
  <video width=650px height=480px muted autoplay loop>
    <source src="https://cdn.monodrive.io/scenario_editor.mp4" type="video/mp4">
  </video>
</div> 

## Placing monoDrive Vehicles in a Scene

The first step in creating a scenario is setting up the monoDrive vehicles in 
the desired scene and applying driving properties to each. To place a vehicle:

1. In the "Content Browser" navigate to the folder `Content/Vehicles`
    <div class="img_container">
      <img class='wide_img' src="../imgs/vehicles_content.png"/>
    </div>

1. Drag one of the vehicles into the scene by click on the vehicle's icon
and putting it on the desired lane of travel.

1. When adding actors that aren't provided by monoDrive to a scenario, make sure to add "serialize" under the Actor's "Tags".
  <div class="img_container">
    <img class='lg_img' src="../imgs/serialize_tag.png"/>
  </div>


1. In the "Details" window, search for the "Vehicle Controller Settings" group 
and and set the desired vehicle controller settings. 
    <div class="img_container">
      <img class='lg_img' src="../imgs/vehicle_controller_settings.png"/>
    </div>

1. In the "Details" window, search for the "Vehicle Behavior Settings" group 
and set the desired vehicle behaviors for traffic laws.
    <div class="img_container">
      <img class='lg_img' src="../imgs/vehicle_behavior_settings.png"/>
    </div>

1. At the top of the Scenario Editor hit the "Play" button. The vehicle should 
begin to travel down the lane closest to its original placement. 

1. Repeat this with multiple vehicles until the scene is populated as desired.

## Detailed Vehicle Behavior Settings

The "Vehicle Settings" section of a vehicle's details contains two types of 
behavior settings: vehicle controller settings and vehicle behavior settings. 

### Vehicle Controller Settings
The first section, "Vehicle Controller Settings", 
contains several settings relative to vehicle behavior for speed and control:

- **Set Speed:**  Sets the desired speed of the vehicle as a delta above 
or below the road's speed limit in miles per hour.
For example, if the road's set speed limit is 40mph, a 
value of "0" would result in the vehicle moving at a speed of 40mph, and with 
a value of "-20", the vehicle's speed would be 20mph.
When the "Obey Road Speed Limits" is not set, the input in this field sets the 
speed of the vehicle (in miles per hour) without regarding the road's speed limit.

- **Initial Speed:** Sets the initial speed of the vehicle as a delta above 
or below the road's speed limit in miles per hour. When the "Obey Road Speed Limits" 
is not set, the input in this field sets the speed of the vehicle (in miles per hour)
 without regarding the road's speed limit.
 
- **Show Vehicle Charts:**  If set, then vehicle dynamics charts will be shown.
	Charts display information about Steering, Speed, Acceleration, Gear, Engine RPM,
	and Throttle. To the right of the charts, the speed of the vehicle is displayed in
	miles per hour.
	<div class="img_container">
      <img class='wide_img' src="../imgs/vehicle_charts_3.png"/>
    </div>

- **Debug Drawing:**  If set, then debug vehicle information will be drawn in the editor. The blue point indicates the steer forward position, the red points indicate the path points, and the cyan point is the predicted distance given the change in speed control. 

	<div class="img_container">
      <img class='lg_img' src="../imgs/debug_draw.png"/>
    </div>

- **Search Forward Distance:**  The number of centimeters to look ahead when computing AI actions. If speeds are much higher than 70mph consider increasing.

- **Search Backward Distance:** The number of centimeters to look backward when computing AI actions.

- **Steer Forward Distance:**  The range of centimeters to look ahead when performing steering AI actions, interpolated based on `steerForwardSpeedRange`.

- **Steer Forward Speed:** The range of speed (mph) used to LERP `steerForwardDistance`.

- **Rolling stop speed:** The speed to travel before continuing after a legal stop.

- **Distance to Stop Sign:**  The number of centimeters to stop before a controlled intersection.

- **Spawn Offset:** The offset to apply to the vehicle position prior to spawning.

- **Deceleration Constraints:** The minimum and maximum deceleration of the vehicle used by AutoPilot. cm/s^2.

- **Turn Pref:** (Turn Preference) At the next turn the vehicle will prefer to take this turn. 
	This will reset the drop down menu to "NONE" after the following turn.

- **Simple Speed Look Ahead:** The number of centimeters to look ahead when performing simple speed control.

- **Vehicle Length:** Total length of the vehicle, used in various maneuver calculations.

- **Point PID:** PID Controller settings for point control.

- **Speed PID:** PID Controller settings for speed control.

- **Lane Offset:**  The number of centimeters to drive from the lane center: positive is right, negative is left.

- **Changing Lanes Left: / Changing Lanes Right:**  Status change only, shows when vehicle is in lane change.

-	**Lane Change Transition Distance:** Lane Change is considered complete after the origin vehicle coordinate frame is this many centimeters from the new lane center.

-	**Steer Forward Distance During Lane Change:** The range in centimeters to look ahead when performing steering AI actions during a lane change, interpolated based on steerForwardSpeedRange.

- **Clamp Decay Exponent:** Exponential decay to lighten the clamping to full steering extents as the vehicle converges on the center lane to regain steering control.

-	**Lane Change Comfort Proximity:** The minimum proximity in centimeters behind and in front of the vehicle for which it is acceptable to change lanes. In addition, the vehicle calculates if it or the other vehicle could stop in time to avoid the collision, and this value sets an extra barrier. 

- **Lane Change Steering Clamp Curve:** Sets a threshold as a function of speed for the steering control. At the start of lane transition, this value will be used, but as the vehicle converges the clamp decay function will take over until full steering control is returned. Lower values smooth the lane change, higher values create a more aggressive lane change. 


### Vehicle Behavior Settings
The "Vehicle Behavior Settings" section of a vehicle's settings contains options for 
obeying (or not) traffic laws:

- **Ignores Lane Change Boxes:**  If set, all lane change trigger boxes will be ignored.

- **Ignore Other Vehicles:**  If set, all other vehicles will be ignored when computing AI actions.

- **Ignore Traffic Signals:**  If set, this vehicle will ignore all traffic control devices.

- **Obey Road Speed Limits:** If set, this vehicle will obey the speed limits of the road and
set speed and initial speed will be a delta above or below the road's speed limit. 

- **Limit Speed in Curves:**  If set, this vehicle will attempt to slow down in order to properly corner in turns.

- **Initial Speed is Delta:** If set, when vehicle is obeying road speed limits, the initial velocity will be used as a delta above or below the speed limit. If vehicle is not obeying road speed limits this setting is ignored.

- **Destroy on Collision:**  If set, this vehicle will be removed in the scene after colliding with another physics body.

- **Should Overtake Slower Vehicles:** If set, will allow vehicle to overtake slower vehicles that it is following for the overtake timer trigger set amount of time. Otherwise, the vehicles will slow to the lead vehicle's speed and remain there. 

- **Overtake Timer Trigger:** The time in seconds a vehicle should wait before triggering the "Should Overtake Slower Vehicles" behavior when it is following a vehicle. Only applies when the vehicle is allowed to overtake vehicles.

- **Snap to Lane:**  If set, this vehicle will search for the nearest lane and snap to it at spawn.

- **Trigger Sleep Time:** A vehicle might hit a dynamic trigger box more than once as they are in motion, to avoid double triggering, behavior can only be modified by the same trigger component again after this many seconds.

### Replay Vehicle Path

The user may specify the vehicle's blueprint path for replay. This path needs to point to a vehicle derived from Custom Physics Vehicle, which can be a shell with no physics implemented for this purpose. 

## Placing Custom Actors and Blueprints in a Scenario

Any actor can be added as part of a scenario by adding the **"serialize"** tag to the actors tag list. See the [Tagging System](#tagging-system) section. Blueprints, Static-Mesh actors, Skeletal-Mesh actors, and Pedestrians are currently supported.

### Blueprint parameters

**"blueprint_parameters"** 

In addition to custom actors, blueprint parameters are also automatically discovered and added to scenario files. These parameters will show up under the **"blueprint_parameters"** key for the given actor. Supported parameter types are are Int32, Float, and Boolean.

## Attached components, Adding Static or Skeletal Meshes to an Actor

**"instance_components"**

Components added to actors or blueprints can be added and saved as part of the scenario as well. This allows you to attach arbitrary objects to scenario actors. For example, you can place tools in a truck bed or a human driver in a vehicle. These components show up under **"instance_components"** and they are automatically discovered by the monoDrive scenario engine. The location in the hierarchy of the actor's components will be preserved.

  <div class="img_container">
    <img class='lg_img' src="../imgs/Add_Component_to_Actor.gif"/>
  </div>
  <!-- docs/imgs/Add_Component_to_Actor.gif -->

## Materials as part of the Scenario

**"tagged_components"**

By default, materials are not serialized to the scenario file. However, if you wish to make the materials changeable from scenario file you can *tag* the component you wish to serialize the materials. This will result in the materials saved out for that component and can be changed in the json to spawn different materials when the scenario is loaded. To do do this add the "serialize" tag to the actor component you want to record the materials for.

  <div class="img_container">
    <img class='lg_img' src="../imgs/Material_Scenario.gif"/>
  </div>

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

## Scenario Setup/ Scenario File Tool Widget

By opening the Scenario File Tool Widget, users can save scenarios to monoDrive scenario files that can then be used in the Closed loop modes of the monoDrive Simulator and Scenario Editor. New Scenario files can be sent to the running simulator to replay dynamic actors for testing and review. 

To begin, find the tool **"Scenario File Tool Widget"** under the Content Folder, right-click and select Run Editor Utility Widget.

  <div class="img_container">
    <img class='lg_img' src="../imgs/scenario_tool_widget.png"/>
  </div>

In the "Scenario File Tool Widget", the user can load or save a new scenario file. Selecting "Save" will write the scene's current configuration to the specified file on FilePath. This will overwrite any file that has the same name.

  <div class="img_container">
    <img class='lg_img' src="../imgs/scenario_tool_widget2.png"/>
  </div>

 It is important to Clear All assets on the editor before using any client to playback the scenario. Failing to do that, may cause issues with the playback.