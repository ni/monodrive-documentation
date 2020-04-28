# Introduction

## Making Autonomous Vehicles Drive Safe
Generation of realistic scenarios and resulting sensor information for perception and planning system validation.

- Low latency networked vehicle control

- Parameterized vehicle dynamics models with run time physics modification

- Fast scenario regeneration, with super realtime replay

- Parameterized and accurate real time sensor emulation

<p>&nbsp;</p>

## Products

**[Simulator](Simulator.md)**

The monoDrive Simulator provides users with the ability to test AV algorithms in one of several pre-made monoDrive Simulator levels. The Open Source monoDrive clients provide an API and examples for connecting to the Simulator for simulating traffic scenarios and replaying monoDrive trajectory files. The clients allow users to configure and test any number of configurations of monoDrive high-fidelity sensors.

The weather system in the monoDrive simulator is an extension of [CARLA's](https://carla.org/) weather implementation. 

**[Scenario Editor](scenario_editor/scenarios.md)**

The monoDrive Scenario Editor extends the monoDrive Simulator and allows the user to control any of the thousands of features available in the monoDrive simulation environments. With the Scenario Editor, users are able to generate custom scenarios leveraging the monoDrive lane and vehicle AIs. Custom simulator maps, vehicle parameters, and driving behaviors can be generated in the Scenario Editor all of which can be used to test AV algorithms using any one of the monoDrive clients.

**[Real to Virtual](r2v/about.md)**

The monoDrive Real-to-Virtual hardware and software provide an end-to-end solution for collecting real-world data from cameras, LiDAR, and GNSS systems to create high-fidelity assets and maps usable in the Unreal Engine and the monoDrive Scenario Editor.

<p>&nbsp;</p>


## How It Works

monoDrive’s platform is designed to automate scenario test generation for planning testing. 

<div class='img_container'>
    <img class="lg_img" src=https://static.wixstatic.com/media/1f1c9e_58512d3c803847989161a59ec21116a6~mv2.png/v1/fill/w_641,h_632,al_c/1f1c9e_58512d3c803847989161a59ec21116a6~mv2.png alt="monoDrive HowItWorks"/>
</div>




<p>&nbsp;</p>
