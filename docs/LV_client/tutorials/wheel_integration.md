# Wheel Integration
The monoDrive LabVIEW client enables users to plug-in a **G920 for Xbox One and PC** wheel in the simulation. MonoDrive's client provides two examples for use with a wheel.   

## **mono_closed_loop_example_wheel.vi**   
This example allows the user to drive the ego car using the wheel providing a better driving experience.

<div class="img_container">
  <video width=650px height=440px muted controls autoplay loop>
    <source src="http://cdn.monodrive.io/readthedocs/closed-loop.mp4" type="video/mp4">
  </video>
</div>

## **mono_replay_example_wheel.vi**   
This example allows the user to move the wheel based on the steering angle calculated from the ego vehicle's yaw.

<div class="img_container">
  <video width=650px height=440px muted controls autoplay loop>
    <source src="http://cdn.monodrive.io/readthedocs/replay.mp4" type="video/mp4">
  </video>
</div>
## Prerequisites
1. Ensure you have the latest drivers for the wheel from [Logitech G HUB](https://www.logitechg.com/en-us/innovation/g-hub.html)   

1. Download the Logitech Steering Wheel SDK from [here](https://www.logitechg.com/en-us/innovation/developer-lab.html)    

1. Create a LabVIEW library using the **LogitechSteeringWheelEnginesWrapper.dll** using the Import Shared Library Wizard, you can find a tutorial on how to do this [here](http://zone.ni.com/reference/en-XX/help/371361R-01/lvhowto/example_import_shared_library/).    

1. Install the **Maker Hub Interface for Xbox One Controller** using the VI Package Manager.   

## How To Run
1. Connect the wheel to the computer. 

1. Open the **mono_closed_loop_example_wheel.vi** or the **mono_replay_example_wheel.vi** 

1. Run the selected VI.


More information about the **G920 for Xbox One and PC** wheel [here](https://www.logitechg.com/en-us/products/driving/driving-force-racing-wheel.html).
<p>&nbsp;</p>
