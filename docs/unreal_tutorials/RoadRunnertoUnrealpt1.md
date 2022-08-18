# monoDrive: RoadRunner to Unreal Pipeline 1

### Overview

These walkthroughs go through each of the monoDrive methods of bringing the maps created in RoadRunner into Unreal and to get the
 For more information, see [monodrive Documentation](https://monodrive.readthedocs.io/en/latest/).

For [Unreal Engine Tutorials](https://monodrive.readthedocs.io/en/latest/unreal_tutorials/MayaWheels/) for guides.

For [Maya Tutorial](https://knowledge.autodesk.com/search-result/caas/simplecontent/content/maya-1-minute-startup-movies.html) for guides.

To continue with the Unreal importation continue to the RoadRunnertoUnreal Part 2.

#### Roadrunner Basics

Overview

This section of the walkthrough goes through the meathods of creating basic roads in RoadRunner as well as how to export them in preperation for Unreal Engine 4.

NOTE: RoadRunner uses Meters as its base measurement scale. Importing a vehicle will help you with the scale of your scene.

#### Road Creation Basics

- Open RoadRunner
    - Create a new scene/map:
  <div class="img_container">
    <img class='lg_img' src="../imgs/RoadRunnerPic_01.png"/>
  </div>

- With the 'Road Plan Tool' selected, located in the top left of the screen:

<div class="img_container">
    <img class='sm_img' src="../imgs/RoadRunnerPic_02.png"/>
</div>

- Navigate to the 'Library Browser' in the bottom right of the screen
    - Navigate to the folder labeled 'Road Styles'
    - Select the road type you would like to use

<div class="img_container">
    <img class='lg_img' src="../imgs/RoadRunnerPic_03.png"/>
</div>

- Now with the 'Road Plan Tool' selected
  1. Right click anywhere on the grid that you see, this will create a red dot
  2. Next 'right' click again in another location, this will create your basic road.

####

#### Quickly creating land

  - With your road drawn, it is time to put some landscape around your road.

- First as the image shows, select the 'Surface Tool'

<div class="img_container">
    <img class='sm_img' src="../imgs/RoadRunnerPic_04.png"/>
</div>

- Now in the screen space right click on the outside of where your road is located.
  - Then make some additional clicks around your road like so:

<div class="img_container">
    <img class='lg_img' src="../imgs/RoadRunnerPic_05.png"/>
</div>

- Then continue to make clicks around the road until you reach the original dot you placed
  - This will create a landscape around your road

<div class="img_container">
    <img class='lg_img' src="../imgs/RoadRunnerPic_06.png"/>
</div>

####

#### Getting Ready to Export to Unreal

Now that you have a road created, it is time to export to Unreal.

- First as the image shows, go to 'File' → 'Export' → 'Unreal'

<div class="img_container">
    <img class='lg_img' src="../imgs/RoadRunnerPic_07.png"/>
</div>

- Once you click on 'Unreal', another window will pop up
  - First select a place to save your files, it is recommended to create a folder
  - The default setting for exporting your .fbx files should be adiquite.
  - Now 'Export'

<div class="img_container">
    <img class='lg_img' src="../imgs/RoadRunnerPic_08.png"/>
</div>

- Next go to 'File' → 'Export' → ' Opendrive'
  - Select the same location that you put your 'Unreal' export to save your file
  - Hit Export

<div class ='img_container'>  

<div class="img_container">
    <img class='wide_img' src="../imgs/RoadRunnerPic_09.png"/>
</div>

<div class="img_container">
    <img class='md_img' src="../imgs/RoadRunnerPic_10.png"/>
</div>

</div>

To continue with the Unreal importation continue to the RoadRunnertoUnreal Part 2.

INTERNAL - NI CONFIDENTIAL

ni.com