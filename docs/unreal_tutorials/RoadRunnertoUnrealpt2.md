# RoadRunner and Unreal Engine Map Setup
## Overview

These walkthroughs go through each of the monoDrive methods of bringing the maps created in RoadRunner into Unreal Engine and to get the
 For more information, see [monodrive Documentation](https://monodrive.readthedocs.io/en/latest/).

## Importing RoadRunner Exports

Once you have exported your assets from RoadRunner, you are now ready to import those assets into Unreal.

NOTE: If you are just trying to setup a map this walkthrough will still be useful.

1. First Open Unreal
    - With Unreal open create a new map
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_01.png"/>
  </div>
1. Then click on 'Default'
1. Once you've created your new map go to the 'Landscape' folder in the 'Content' browser

<div class="img_container">
    <img class='lg_img' src="../imgs/UnrealPic_02.png"/>
</div>

1. Then click on 'Import' and navigate to your RoadRunner .fbx export and open
1. An 'FBX Import Option' window will open
    - Find the 'Import Uniform Scale' option
    - Type in 100.0
1. Click 'Import All'
1. Click and Drag all assets into the scene
1. Now with all assets selected go to the 'Details' Panel
    - In the 'Location' section in the X,Y,Z section type in a 0 for each slot
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_03.png"/>
  </div>

## Setting up Asset Collision

To make your road drive able you need to adjust the collision of the assets you've imported.

1. First open any of the assets that you imported by double clicking on them
1. Scroll down or search the 'Details' panel for the 'Collision' section
1. Now adjust the 'Collision Complexity' so that it reads 'Use Complex Collision as Simple'
1. Then adjust the 'Collision Presets' so that it reads 'Road' or 'Landscape'
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_04.png"/>
  </div>

1. Repeat on your other imported meshes

## Adjusting Materials to be Sensor Ready

1. Now to make your materials viewable to radar you must attach 'Physical Materials'.
1. Open any of your materials
1. Find the 'Details' panel on the left side of the screen
    - Find where it says 'Phys Material'
    - Click on the drop down where it says 'None'
    - Apply the material that applies most to your material type
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_05.png"/>
  </div>

## Importing OpenDrive

1. With your road and landscape imported it is time to Import your OpenDrive file.
1. If you do not see a tab open in Unreal labeled 'RoadNetworkToolWidget' go to the top level of the 'Content' structure
1. Next right click on the icon that says 'RoadNetworkToolWidget'
1. Now click on 'Run Editor Utility Widget'
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_06.png"/>
  </div>
1. You should see a window that looks like this:
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_07.png"/>
  </div>
1. Click on the 'Load' option
1. Find your exported Opendrive file from RoadRunner
1. Now go to the 'World Outliner'
1. Find the asset named 'OpenDriveRoadNetwork' and click on it
1. Now navigate to the 'Details' panel
1. In the 'OpenDrive' section you should see a button labeled 'Generate OpenDRIVE Mesh'
1. Click on it.
    - NOTE: all this is doing is confirming that your OpenDrive and your road are matching up
1. If everything is done correctly, it should look something like this:
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_08.png"/>
  </div>
1. To get rid of the mesh click on the 'Clear OpenDRIVE Mesh'

## Weather

To quickly apply weather to your new level follow these steps.

1. First go to the top left of Unreal and find 'Window' → 'Level's
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_09.png"/>
  </div>
1. Next go to the 'Content' Browser to the 'Maps' → 'SubLevels' folder
  - Make a folder with your maps name as the title
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_10.png"/>
  </div>
1. Next, open the 'Straightaway5k' folder
1. Copy the 'straightaway5k\_Weather' sublevel to your levels sublevel folder
1. Next rename the file to "YourLevelNameHere\_Weather"
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_11.png"/>
  </div>
1. Once this is done, click and drag the sublevel over to the 'Levels' tab you opened earlier
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_12.png"/>
  </div>
1. Right click on the sublevel and go to 'Change Streaming Method' → 'Always Loaded'
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_13.png"/>
  </div>
1. Next go back to the 'World Outliner'
    - Delete anything that isn't in the 'Atmosphere' folder and that isn't your Imported assets.
    - It should look like this:
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_14.png"/>
  </div>

## Static Assets

Now it is time to populate your environment.

## Placing Road Signs

1. All of the signs in the Content → Meshes → Signs Folder are mesh templates
1. All of the sign materials in the Content → Materials → Signs Folder will match with their corresponding names.
1. Click and Drag the the sign you would like to use from the 'Content' browser into the viewport, ex: Diamond\_36\_Sign
1. In the details panel → Materials there will be an empty 'Element' slot (1.a)
  <div class="img_container">
      <img class='md_img' src="../imgs/UnrealPic_15.png"/>
  </div>1.a
1. Navigate to the Content → Materials → Signs Folder, pick the appropriate sign shape folder, ex: Diamond\_Signs, and open the folder
1. Find the material that sign is needed.
1. Click and drag the material in to the empty 'Element slot'
1. To find a short video on this subject [click here](https://monodrive.readthedocs.io/en/latest/unreal_tutorials/PlacingSigns/) to go to the monoDrive read the docs section about it.

## Setting Custom Depth Stencil Values

Use this tutorial to set up Custom Depth Stencil Values.

### Application:

1. As stated above, place a static mesh into your scene
1. Now with the mesh selected, locate the Details panel
    - Type 'custom' into the search bar (2.a)
1. Check the box labeled 'Render CustomDepth Pass'
1. Go to the link provided and fill in the Custom Depth Stencil Value that matches your static mesh the closest.
    - [Data Products — monoDrive Simulator](https://monodrive.readthedocs.io/en/latest/r2v/data_processing_output/)
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_16.png"/>
  </div>(2.a)

## Setting up Collision for Static Meshes

Use this tutorial to set up the collision.

### Application:

1. Open up your static mesh
    - In the middle of the screen you should see the 'Collision' option
  <div class="img_container">
      <img class='sm_img' src="../imgs/UnrealPic_17.png"/>
  </div>
1. There should be a green box around your static mesh
    - If not, look in the top right of your screen and find the 'Collision' drop down menu
    - You can try any of the collision options, but a good starter is the box collision
1. Now in the 'Details' panel on the right
    - Search collision
    - You will want to set the 'Collision Presets'
    - Select the preset that match's your object the best
    - For most static meshes the 'BlockAll' option is best
    - Now in the 'Details' panel on the right
    - For any landscape static meshes select the 'Landscape' option


## Spline Assets

### Placing Mesh Splines

1. Now in the 'Details' panel on the right
    - Find the blueprint called 'Spline\_Actor'
1. Drag said blueprint into the viewport
    - This will already have the 'concrete barrier' asset assigned to the Mesh slot (1.b)
  <div class="img_container">
      <img class='md_img' src="../imgs/UnrealPic_18.png"/>
  </div>
1. There will be two spline points near the 'concrete barrier'
    - Click and drag the spline point the 'Translate', 'Rotate' or 'Scale tool is not over. (1.c)
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_19.png"/>
  </div>
1. Now replace the 'concrete barrier' asset with a static mesh of your choosing
    - The Mesh slot is located in the Detail Panel
    - Side note: you can use flowing assets like the concrete barrier/guardrails or you can place repeatable individual assets like the street lights/Light posts.
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_20.png"/>
  </div>
1. Now adjust the 'Spacing' value in the details panel
    - Side note depending on whether you are adjusting a flowing asset or a repeatable asset, the 'Spacing' value can vary greatly.
    - Rule of Thumb: the larger the 'Spacing value the more space apart the asset will be
1. A good starting point for 'flowing' assets is: 200-600
1. A good starting point for repeatable assets is: 2000-5000
1. To find a short video on this subject [click here](https://monodrive.readthedocs.io/en/latest/unreal_tutorials/SingularMesh/) to go to the monoDrive read the docs section about it.

### Spline Curving

1. When needing to curve the spline asset do the following
    - For basic curves, double click on either of the 2 original spline points with your rotation tool selected
    - You will see two 'arms' display themselves, you will now be in the proper mode
1. Now use the rotate tool to curve the spline (2.a)
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_21.png"/>
  </div>
1. For more complex curves, right click on the spline itself and click 'Add Spline Point Here'
1. As before, double click on the spline point
    - Use the 'Translate' and 'Rotate tool to adjust the spline into the position you require.
1. To find a short video on this subject [click here](https://monodrive.readthedocs.io/en/latest/unreal_tutorials/SplineMesh/#adjusting-a-spline-mesh) to go to the monoDrive read the docs section about it.

## Foliage Assets

### Painting Trees & Foliage Setup

1. To paint foliage into your scene you must go to the 'Foliage' mode located in the top left corner of Unreal. (Note this is true for all Unreal's up to 4.24) (3.a)
  <div class="img_container">
      <img class='sm_img' src="../imgs/UnrealPic_22.png"/>
  </div>(3.a)

1. To add foliage to the 'Foliage' Painter tool, navigate to where your foliage content is. Ex: Content → Foliage → Trees → RedMaple
    - If you already have a foliage actor, indicated by the green line at the bottom of the asset drag it into the 'Drop Foliage Here' section as seen in 3.a.
    - If you don't already have a foliage actor, click and drag your static mesh into the 'Drop Foliage Here' section and Unreal will create one for you. (4.a)
  <div class="img_container">
      <img class='sm_img' src="../imgs/UnrealPic_23.png"/>
  </div>
1. Now hover your mouse over the new foliage actor and check the empty box and select the tree.
1. To find a short video on this subject [click here](https://monodrive.readthedocs.io/en/latest/unreal_tutorials/FoliagePainting/) to go to the monoDrive read the docs section about it.

### Adjusting the Foliage Settings

A good starting point is something like this (5.a):

<div class="img_container">
    <img class='md_img' src="../imgs/UnrealPic_24.png"/>
</div>

### Setting Collision for Grasses and Bushes

1. Foliage actors such as small bushes, grasses and other small foliage's collision setting should look as so (6.a) :
  <div class="img_container">
      <img class='md_img' src="../imgs/UnrealPic_25.png"/>
  </div>(6.a)

1. Foliage actors such as large bushes, boulders and trees should have their collision setting set as so (7.a):
  <div class="img_container">
      <img class='md_img' src="../imgs/UnrealPic_26.png"/>
  </div>(7.a)

### Setting Collision for Trees

1. Navigate to the static mesh of your tree
    - Scroll down the 'Details' panel until you find the 'Collision' section
    - Find the option 'Collision Complexity'
    - Set the 'Collision Complexity' to 'Simple and Complex'
1. Next, change your 'Collision Presets' to 'Custom'
    - Set 'Radar' to 'Ignore'
    - Set all the other settings to 'Block'
1. If everything is set properly it should look like this (8.a) :
  <div class="img_container">
      <img class='wide_lg_img' src="../imgs/UnrealPic_27.png"/>
  </div>(8.a)

### Setting your Physical Materials for Foliage

1. Navigate to the materials that your tree is using
    - Open the various materials
1. In the materials 'Details' panel navigate to the 'Physics Materials' section
1. Click the drop down menu
    - Select the 'Tree' physical material or the physical material that matches best to your object. (9.a)
  <div class="img_container">
      <img class='sm_img' src="../imgs/UnrealPic_28.png"/>
  </div>(9.a)

### Painting your Trees

To paint your trees first you must set up your brush settings:

1. So to not overload your scene with too many trees start with a low 'Paint Density' setting, For example: .1
1. Under the 'Filters' section have only the kinds of objects that you want to be painting on selected. For example: Landscape or Static Meshes.
1. Now that you have everything set move your mouse into the viewport, you should see a semi-transparent grey dome.
1. Now 'Left' click in the viewport, you should see trees start to pop up.
    - If need be, adjust the settings that were talked about above to adjust how many trees are being painted.
    - To erase trees hold down 'shift' and left click and hold over the trees you want to erase.