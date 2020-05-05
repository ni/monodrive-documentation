# Wheel integration

## Pre-requisites
1. The **mono_closed_loop_example_wheel.vi** and the **mono_replay_example_wheel.vi** use the **G920 for Xbox One and PC**, [more information](https://www.logitechg.com/en-us/products/driving/driving-force-racing-wheel.html)
1. Download the Logitech Steering Wheel SDK from [here](https://www.logitechg.com/en-us/innovation/developer-lab.html)
1. Create a LabVIEW library using the **LogitechSteeringWheelEnginesWrapper.dll** using the Import Shared Library Wizard, you can find a tutorial on how to do this [here](http://zone.ni.com/reference/en-XX/help/371361R-01/lvhowto/example_import_shared_library/).
1. Install the **Maker Hub Interface for Xbox One Controller** using the VIPM.


## Closed Loop with Wheel Block Diagram
<div class="img_container">
    <img class="lg_img" src="https://github.com/monoDriveIO/documentation/raw/wheel_integration/docs/LV_client/tutorials/closed_loop_connections.png"/>
</div>

### Demo
<div class="img_container">
  <video width=650px height=440px muted controls autoplay loop>
    <source src="http://cdn.monodrive.io/readthedocs/closed-loop.mp4" type="video/mp4">
  </video>
</div>


## Replay with Wheel Block Diagram
<div class="img_container">
    <img class="lg_img" src="https://github.com/monoDriveIO/documentation/raw/wheel_integration/docs/LV_client/tutorials/replay_connections.png"/>
</div>

### Demo
<div class="img_container">
  <video width=650px height=440px muted controls autoplay loop>
    <source src="http://cdn.monodrive.io/readthedocs/replay.mp4" type="video/mp4">
  </video>
</div>


<p>&nbsp;</p>
