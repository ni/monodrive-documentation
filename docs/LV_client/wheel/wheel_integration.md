# Wheel integration

1. Download the Logitech Steering Wheel SDK from [here](https://www.logitechg.com/en-us/innovation/developer-lab.html).
1. Create a LabVIEW library using the LogitechSteeringWheelEnginesWrapper.dll using the Import Shared Library Wizard, you can find a tutorial [here](http://zone.ni.com/reference/en-XX/help/371361R-01/lvhowto/example_import_shared_library/).
1. Install the `Maker Hub Interface for Xbox One Controller` using the VIPM.
1. Open the `mono_closed_loop_example_wheel.vi` or the `mono_replay_example_wheel.vi` 

## Closed Loop
- Add the `mono_wheel.vi` to your block diagram as shown in the image below.
- Convert the throttle, steering, brake and direction controls to indicators.   
<div class="img_container">
    <img class="lg_img" src="https://github.com/monoDriveIO/documentation/tree/wheel-integration/docs/LV_client/wheel/closed_loop_connections.png"/>
</div>

## Replay
- Add the `mono_turn_wheel.vi` to your block diagram as shown in the image below.
- Obtain the `ego_yaw` from the `mono_gps.vi` and connect to the input of the `mono_turn_wheel.vi`   
<div class="img_container">
    <img class="lg_img" src="https://github.com/monoDriveIO/documentation/tree/wheel-integration/docs/LV_client/wheel/replay_connections.png"/>
</div>

<p>&nbsp;</p>
