# Using Real-to-Virtual Maps

## Using monoDrive Real-to-Virtual Maps in Simulation

This demonstrates how to install and use monoDrive Real-to-Virtual levels
in the monoDrive Scenario Editor. Download and installation instructions for the
Scenario Editor can be [found here](../../Scenario_editor).

### Installing Maps

After Real-to-Virtual data has been processed by the monoDrive Real-to-Virtual
data processing pipeline, the final level is constructed and ready for use in 
the Unreal Engine with the monoDrive Scenario Editor. When the level is 
completed, it will be made available through the monoDrive cloud service in a 
zip file format. To install the level:

* Download the level's zip archive from the monoDrive cloud
* Extract the archive's contents, it will contain a `Content` directory that should be moved into the Scenario Editor's install location.
* Open the extracted directory and locate the directory that contains directories
* Copy all of the contents of this directory and paste them into the installation directory of the Scenario Editor `<Install Directory>/VehicleAI_Editor/Content/`
    * **NOTE:** Choose to merge with existing directories, but do not choose to overwrite existing files

<div class="img_container">
    <img class='wide_img' src="../imgs/copying_real_to_virtual_extracted.png"/>
</div>

<p>&nbsp;</p>

<div class="img_container">
    <img class='lg_img' src="../imgs/copying_real_to_virtual_folder.png"/>
</div>

<p>&nbsp;</p>

<div class="img_container">
    <img class='lg_img' src="../imgs/copying_real_to_virtual_skip.png"/>
</div>
<p>&nbsp;</p>

### Opening and Using Maps

Once the map has been copied to the appropriate location, it will be available 
to load in the Scenario Editor. 

* Browse to the `Content/Maps` directory in the Scenario Editor and find the map in the `Real-to-Virtual` directory.

<div class="img_container">
    <img class='lg_img' src="../imgs/opening_r2v_map.png"/>
</div>
<p>&nbsp;</p>

* Double-clicking the map will load the map. The below image shows an overhead view of a Real-to-Virtual map loaded in the Scenario Editor.

<div class="img_container">
    <img class='wide_img' src="../imgs/r2v_map_overhead.png"/>
</div>
<p>&nbsp;</p>

#### Playback of Trajectory

If the Dynamic Actors were created for the Real-to-Virtual data collection, 
packaged with the map should be a `replay.json` file that contains all detected
Dynamic Actors in the scene. This file is a monoDrive Trajectory File that can
be played back in the Scenario Editor using the "Replay Mode" of any of the 
monoDrive clients. Below is a an image of the trajectory being played back using
the monoDrive LabView Client. See the 
[LabView Client Replay Example](../../LV_client/quick_start/LabVIEW_client_quick_start)
for more information about playing back Trajectory Files.

<div class="img_container">
    <img class='wide_img' src="../imgs/r2v_labview_client.png"/>
</div>
<p>&nbsp;</p>

#### Creating new Trajectory Files

The Real-to-Virtual map can also be used the same as any other map packaged with
the monoDrive Scenario Editor. See 
[the documentation](../../Scenario_editor) 
for creating Trajectory Files in the monoDrive Scenario editor for more 
information on using the monoDrive maps.

<div class="img_container">
    <img class='wide_img' src="../imgs/r2v_simulator_recording.png"/>
</div>
<p>&nbsp;</p>
