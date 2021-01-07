# Vehicle Behavior Trigger Boxes

Vehicle Behavior Trigger Boxes define specific actions or behaviors to happen 
once that trigger box interacts with another vehicle or intersection in the scene. 
The action can have an effect on the vehicle with the trigger box or the vehicle it is interacting 
within the scene. 

<div class="img_container">
    <img class='md_img' src="../imgs/trigger_scene1.png"/>
    <div class="space"></div>
    <img class='md_img' src="../imgs/trigger_scene2.png"/>
</div>


## Adding a Trigger Box

1. Click the vehicle and then go the "Details" window

1. In the vehicle's "Details" window, click the "Add Component" button at the top and search for "Vehicle Behavior Trigger."
  <div class="img_container">
    <img class='lg_img' src="../imgs/trigger_details1.png"/>
  </div>

1. Select the "Vehicle Behavior Trigger" component from the list to attach it.

1. Click on the `VehicleBehaviorTrigger` component in the vehicle's hierarchy in side of the "Details" window to bring up the settings for adding trigger boxes.

1. To add a trigger box, select the plus icon next inside the "Trigger Boxes" Group.
  <div class="img_container">
    <img class='lg_img' src="../imgs/trigger_details2.png"/>
  </div>

1. Choose the desired settings for the vehicle or other vehicles behavior
in relation to this vehicle.
  <div class="img_container">
    <img class='lg_img' src="../imgs/trigger_box4.png"/>
  </div>

## Vehicle Trigger Box Properties

  - **Distance:** The distance forward or backward (+/-) from the vehicle to place the trigger box. 

  - **Lane:** The number of lanes to shift the trigger box. 
  Negative numbers indicate lanes to the left of the vehicle, and positive to the right. 

  - **Scale:** How large the trigger box is.

  - **Owner:** The description of the actions that this box will trigger on the owning vehicle.

  - **Other:** The description of the actions that this box will trigger on the other vehicle.

## Vehicle Trigger Actions
  - **Change Lane Left:** Triggers the vehicle to execute a lane change to the left lane if one is available. 

  - **Change Lane Right:** Triggers the vehicle to execute a lane change to the right lane if one is available. 

  - **Can Stay in Lane:** If selected and multiple lane change options are selected the vehicle may chose to stay in its lane. 

  - **Next Turn Left:** Triggers the vehicle to take the leftmost lane at the next intersection or lane branch. 

  - **Next Turn Right:** Triggers the vehicle to take the rightmost lane at the next intersection or lane branch.

  - **Next Turn Center:** Triggers the vehicle to take the center lane at the next intersection or lane branch. 

  - **Number of Lanes to Change:** If a lane change is selected, the number of lanes the vehicle should try to change.

  - **Speed Change:** The new speed the vehicle should set point to. Only takes effect if ShouldSpeedChange is set to true. (mph) 

  - **Should Speed Change** Whether the speed change value should be used on vehicle.

## Debug
  - **Debug Draw:** A translucent box will appear where the trigger box is located 
  as a debug visualization.
