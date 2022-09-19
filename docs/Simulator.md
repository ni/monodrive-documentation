# Simulator 

The monoDrive Simulator is a powerful simulation software featuring high-fidelity sensor models, realistic driving scenarios, and diverse environments. The Open Source monoDrive clients provide an API and examples for connecting to the Simulator for simulating traffic scenarios and replaying monoDrive trajectory files. The clients enable users to configure and test any number of configurations of monoDrive high-fidelity sensors. For more information on the each monoDrive client, see the quick start pages: [C++ Client](../cpp_client/cpp_quick_start), [LabVIEW Client](../LV_client/quick_start/LabVIEW_client_quick_start), and [Python Client](../python_client/quick_start).

The Simulator offers three modes for robust testing. Note, at one point there were four modes. The replay, closed loop, and HIL modes have all been collapsed into one very flexible hybrid mode allowing users to mix replay actors, closed loop actors, and HIL functionality all at the same time!

- *Continuous*, the clock of monoDrive is real-time and driven by the local machines wall clock on which the instance is running. State driven or replay actors can still be driven but all timing for local physics actors are controlled by monoDrive clock. "simulator_mode": 0

- *Fixed Step*, the clock of monoDrive only progresses when a step command is received from the client. Time is otherwise frozen until a step command is received. Replay and state actors actors can still be moved via client commands. "simulator_mode": 3

- *Remote Clock*, the clock of monoDrive is controlled externally through the UpdateState command. Local physics actors (if used in this mode) are still controlled by the local monoDrive clock, however, but all time reporting will be done with respect to the external clock input. "simulator_mode": 1

Through a monoDrive Client, users may control specific vehicle behavior. These examples show vehicle behaviors for three different set-ups on the monoDrive Almono Map.


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