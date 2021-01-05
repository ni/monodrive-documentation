# Simulator 

The monoDrive Simulator is a powerful simulation software featuring high-fidelity sensor models, realistic driving scenarios, and diverse environments. The Open Source monoDrive clients provide an API and examples for connecting to the Simulator for simulating traffic scenarios and replaying monoDrive trajectory files. The clients enable users to configure and test any number of configurations of monoDrive high-fidelity sensors. For more information on the each monoDrive client, see the quick start pages: [C++ Client](../cpp_client/cpp_quick_start), [LabVIEW Client](../LV_client/quick_start/LabVIEW_client_quick_start), and [Python Client](../python_client/quick_start).

The Simulator offers four modes for robust testing: 

- *Closed Loop*, actively control the system based on sensor and vehicle feedback.

- *Replay*, executes a fixed sequence with controlled vehicle positions without any feedback.

- *HIL*, hardware in the loop, enables users to connect hardware for perception and control and integrate them into various pieces of a closed loop or replay simulation.

- *Closed Loop with Fixed Step*, enables a user to control the time delta between simulation steps regardless of the wall clock time.

## Maps

Through a monoDrive Client, users may control specific vehicle behavior. These examples show vehicle behaviors for three different set-ups.

### Almono Map


<div class="img_container">
  <video width=650px height=480px muted autoplay loop>
    <source src="https://cdn.monodrive.io/readthedocs/highway_exit_trajectory.mp4" type="video/mp4">
  </video>
</div> 

<div class="img_container">
  <video width=650px height=480px muted autoplay loop>
    <source src="https://cdn.monodrive.io/readthedocs/sudden_stop_trajectory.mp4" type="video/mp4">
  </video>
</div> 

<div class="img_container">
  <video width=650px height=480px muted autoplay loop>
    <source src="https://cdn.monodrive.io/readthedocs/traffic_circle.mp4" type="video/mp4">
  </video>
</div> 

### Straight Highway

These are two examples of the Simulator on the Straight Highway Map. The first demonstrating a car following the right lane, and the second displaying a quick lane change.

<div class="img_container">
  <video width=650px height=480px muted autoplay loop>
    <source src="https://cdn.monodrive.io/readthedocs/straight_highway.mp4" type="video/mp4">
  </video>
</div> 

<div class="img_container">
  <video width=650px height=480px muted autoplay loop>
    <source src="https://cdn.monodrive.io/readthedocs/highway_lane_change.mp4" type="video/mp4">
  </video>
</div> 

### Integrate Carla Maps

Two Carla maps Town04 and Town07 have been added to monoDrive. These maps have been updated with monoDrive physical materials, lighting, semantic labeling, dynamic weather, traffic signaling, and ray tracing settings.

To see a tutorial on how to import a Carla Map, See [Carla to monoDrive Conversion](https://docs.google.com/document/d/18JTYXOFPb98qLm8aHvCYH4nb2eU6P5sVzD1qkONqTeU/edit).

<p class="img_container">
<img class="wide_img" src="../imgs/tractor.png"/>
</p>

<p class="img_container">
<img class="wide_img" src="../imgs/town.png"/>
</p>

<p>&nbsp;</p>