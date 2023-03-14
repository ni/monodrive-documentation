# LabVIEW Client Quick Start

## Prerequisites 

1. Register and Download [LabVIEW 2019 (64 bit)](https://www.ni.com/en-us/support/downloads/software-products/download.labview.html#329483) *approximately 1 hour to download*

1. Open the monoDrive Simulator or monoDrive Simulator Scenario Editor

    monoDrive Simulator

    - Powerful tool for testing AV algorithms using Open Source monoDrive Clients or monoDrive Simulator API. 

    monoDrive Simulator Scenario Editor

    - Take AV algorithm testing to the next level by using the rich Scenario Editor tool for creating custom simulations and environments.


1. The monoDrive LabVIEW Client is Open Source Software for connecting to and 
configuring the monoDrive Simulator and Scenario Editor. To get started, 
user may clone the client from the monoDrive GitHub repository or 
through LabVIEW's VI Package Manager. 

    - To clone the client through GitHub, please contact us directly at monodrive.support@ni.com

    - To get access through the VI Package Manager, open the VI Package Manager as Administrator 
    and install the monoDrive Client.
    <div class="img_container">
        <img class='lg_img' src="../imgs/prereq.png"/>
    </div>

<p>&nbsp;</p>


## Run the VehicleAI Simulator

1. Go to your VehicleAI directory and find VehicleAI.exe

2. Double-click on VehicleAI.exe

3. Move to one side of your screen

<div class="img_container">
    <img class='lg_img' src="../imgs/runVehicleAI.png"/>
</div>

<p>&nbsp;</p>


## Run the VehicleAI Scenario Editor
Quick start instructions and details on how to generate project files in the VehicleAI Scenario Editor can be found in [Getting Started Guide](../../Getting_Started.md).

1. Go to your VehicleAI_Editor directory and find VehicleAI.uproject


1. Double-click on VehicleAI.uproject

<p>&nbsp;</p>

## Open the LabVIEW Client

1. Open the LabVIEW CLient
    
    - For the LabVIEW Client cloned through GitHub, you may go directly to the repo folder 
    and open the `ni_monoDrive.lvproj`. 

    - For opening through the VI Package Manager, navigate to the monoDriveClient folder, 
    you can find it on your NI Examples, typically on:

        *C:\Program Files\National Instruments\LabVIEW 2019\examples\monoDrive\monoDriveClient* 

        You can also find the examples on the NI example Finder.

        Navigate to Help -> Find Examples...

        Select Directory Structure

        Expand the monoDrive folder


        <div class="img_container">
        <img class='lg_img' src="../imgs/find_examples.png"/>
        </div>

        <div class="img_container">
        <img class='lg_img' src="../imgs/find_examples2.png"/>
        </div>


        You can also find the examples on the NI example Finder.

2. The client provides the following examples:

    - [mono1_pilot_example.vi](../LabVIEW_run_examples/#pilot-example)
    - [mono2_scenario_example.vi](../LabVIEW_run_examples/#scenario-example)
    - [mono3_replay_example.vi](../LabVIEW_run_examples/#replay-example)
    - [mono4_hil_example.vi](../LabVIEW_run_examples/#hil-example)
    - [mono5_radar_example.vi](../LabVIEW_run_examples/#radar-example)
    - [mono6_pilot_example_fixed_time](../LabVIEW_run_examples/#pilot-example-with-fixed-time)
    - [experimental examples](../LabVIEW_run_examples/#experimental-examples)

3. Double-click on the VI to open the example.


<p>&nbsp;</p>


For technical support contact us at <b>monodrive.support@ni.com</b>
<p>&nbsp;</p>
