# RoadRunner and Unreal Engine Map Setup
## Overview

The following walk-through outlines methods for integrating maps created in RoadRunner into Unreal Engine 4. It continues on to detail how to create a functioning map in Unreal Engine 4.

For more information, see [monodrive Documentation](https://monodrive.readthedocs.io/en/latest/).

## Importing Assets into Unreal Engine

Before beginning, this section outlines the creation and setup of a Unreal Engine map/scene and may apply to assets other than RoadRunner assets.

1. Open Unreal Engine
    - Create a new map
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_01.png"/>
  </div>
1. Click on 'Default'
1. Once the new map is created, go to the 'Landscape' folder in the 'Content' browser
  <div class="img_container">
      <img class='wide_lg_img' src="../imgs/UnrealPic_02.png"/>
  </div>
1. Then click on 'Import' and navigate to the RoadRunner .fbx export and open
1. An 'FBX Import Option' window will open
    - Find the 'Import Uniform Scale' option
    - Type in 100.0
1. Click 'Import All'
1. Click and Drag all assets into the scene
1. With all assets selected, go to the 'Details' Panel
    - In the 'Transform' Section, navigate to 'Location' which has the values for X,Y,Z.  Ensure that X,Y,Z values in the Location each have a value of "0"
  <div class="img_container">
      <img class='wide_img' src="../imgs/UnrealPic_03.png"/>
  </div>

## Setting up Asset Collision

To make the road driveable, the user will need to adjust the collision of the imported assets.

1. Open any of the assets that were imported by double clicking on them
1. Scroll down or search the 'Details' panel for the 'Collision' section
1. Adjust the 'Collision Complexity' so that it reads 'Use Complex Collision as Simple'
1. Adjust the 'Collision Presets' so that it reads 'Road' or 'Landscape'
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_04.png"/>
  </div>

1. Repeat for other imported meshes

## Adjusting Materials to be Sensor Ready

To make any asset viewable to the Radar 'Physical Materials,' it must be attached to materials.

1. Open any material that needs to be adjusted
1. Find the 'Details' panel on the left side of the screen
    - Find where it says 'Phys Material'
    - Click on the drop down where it says 'None'
    - Apply the material that applies most to the material type
  <div class="img_container">
      <img class='wide_img' src="../imgs/UnrealPic_05.png"/>
  </div>

## Importing OpenDrive

With the road and landscape imported it is time to import the OpenDrive file.
1. Go to the top level of the 'Content' structure 
1. Right click on the icon that says 'RoadNetworkToolWidget'
1. Click on 'Run Editor Utility Widget', and the 'Road Network Import/Export Tool' window should open.
  <div class="img_container">
      <img class='wide_lg_img' src="../imgs/UnrealPic_06.png"/>
  </div>
1. A window will open that looks like this:
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_07.png"/>
  </div>
1. Click on the 'Load' option
1. Find the exported Opendrive file from RoadRunner
1. Go to the 'World Outliner'
1. Find the asset named 'OpenDriveRoadNetwork', click on it
1. Navigate to the 'Details' panel
1. In the 'OpenDrive' section, navigate and click on the button labeled 'Generate OpenDRIVE Mesh'
1. Click on it.
    - This confirms that the OpenDrive and the road match up
1. If everything is done correctly, it should look something like this:
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_08.png"/>
  </div>
1. To get rid of the mesh click on 'Clear OpenDRIVE Mesh'

## Weather

To apply weather to the new level follow these steps.

1. Go to the top left of Unreal and find 'Window' → 'Levels'
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_09.png"/>
  </div>
1. Next go to the 'Content' Browser to the 'Maps' → 'SubLevels' folder
  - Make a folder with the maps name as the title
  <div class="img_container">
      <img class='wide_lg_img' src="../imgs/UnrealPic_10.png"/>
  </div>
1. Next, open the 'Straightaway5k' folder
1. Copy the 'straightaway5k\_Weather' sublevel to the levels sublevel folder
1. Next rename the file to "YourLevelNameHere\_Weather"
  <div class="img_container">
      <img class='wide_lg_img' src="../imgs/UnrealPic_11.png"/>
  </div>
1. Once this is done, click and drag the sublevel over to the 'Levels' tab opened earlier
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_12.png"/>
  </div>
1. Right click on the sublevel and go to 'Change Streaming Method' → 'Always Loaded'
1. NOTE: If this is not done, when 'Play' is hit the assets in the level will not appear.
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_13.png"/>
  </div>
1. Next go back to the 'World Outliner'
    - Delete anything that isn't in the 'Atmosphere' folder and that is not an Imported asset.
    - It should look something like this:
    <div class="img_container">
        <img class='lg_img' src="../imgs/UnrealPic_14.png"/>
    </div>

## Static Assets

Now it is time to populate the environment.

## Placing Road Signs

1. All of the signs in the Content → Meshes → Signs Folder are mesh templates
1. All of the sign materials in the Content → Materials → Signs Folder will match with their corresponding names.
1. Click and Drag a sign from the 'Content' browser into the viewport, ex: Diamond\_36\_Sign
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

1. As stated above, place a static mesh into the scene
1. Now with the mesh selected, locate the 'Details' panel
    - Type 'custom' into the search bar (2.a)
1. Check the box labeled 'Render CustomDepth Pass'
1. Go to the link provided and fill in the Custom Depth Stencil Value that matches the static mesh the closest.
    - [Data Products — monoDrive Simulator](https://monodrive.readthedocs.io/en/latest/r2v/data_processing_output/)
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_16.png"/>
  </div>(2.a)

## Setting up Collision for Static Meshes

Use this tutorial to set up the collision.

### Application:

1. Open up a static mesh
    - In the middle of the screen you should see the 'Collision' option
  <div class="img_container">
      <img class='sm_img' src="../imgs/UnrealPic_17.png"/>
  </div>
1. There should be a green box around the static mesh
    - If not, look in the top right of the screen and find 'Collision' drop down menu
    - Try any of the collision options, a good starter is the box collision
1. Now in the 'Details' panel on the right
    - Search collision
    - You will want to set the 'Collision Presets'
    - Select the preset that match's the object the best
    - For most static meshes the 'BlockAll' option is best
      - But try and match it as best as possible .i.e 'landscape', 'road', etc. 
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
1. Now replace the 'concrete barrier' asset with any static mesh
    - The Mesh slot is located in the Detail Panel
    - NOTE: The user can use flowing assets like the concrete barrier/guardrails or you can place repeatable individual assets like the street lights/Light posts.
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_20.png"/>
  </div>
1. Now adjust the 'Spacing' value in the details panel
    - Side note depending on whether the user is adjusting a flowing asset or a repeatable asset, the 'Spacing' value can vary greatly.
    - Rule of Thumb: the larger the 'Spacing value the more space apart the asset will be
1. 'Flowing' assets is: 200-600
1. 'Repeatable' assets is: 2000-5000
1. To find a short video on this subject [click here](https://monodrive.readthedocs.io/en/latest/unreal_tutorials/SingularMesh/) to go to the monoDrive read the docs section about it.

### Spline Curving

1. When needing to curve the spline asset do the following
    - For basic curves, double click on either of the 2 original spline points with the rotation tool selected
    - You will see two 'arms' display themselves, and now will be in the proper mode
1. Now use the rotate tool to curve the spline (2.a)
  <div class="img_container">
      <img class='lg_img' src="../imgs/UnrealPic_21.png"/>
  </div>
1. For more complex curves, right click on the spline itself and click 'Add Spline Point Here'
1. As before, double click on the spline point
    - Use the 'Translate' and 'Rotate tool to adjust the spline into the position required.
1. To find a short video on this subject [click here](https://monodrive.readthedocs.io/en/latest/unreal_tutorials/SplineMesh/#adjusting-a-spline-mesh) to go to the monoDrive read the docs section about it.

## Foliage Assets

### Painting Trees & Foliage Setup

1. To paint foliage into the scene you must go to the 'Foliage' mode located in the top left corner of Unreal. (Note this is true for all Unreal Engine's up to 4.24) (3.a)
  <div class="img_container">
      <img class='sm_img' src="../imgs/UnrealPic_22.png"/>
  </div>(3.a)

1. To add foliage to the 'Foliage' Painter tool, navigate to where the foliage content is. Ex: Content → Foliage → Trees → RedMaple
    - If the user already has a foliage actor, indicated by the green line at the bottom of the asset drag it into the 'Drop Foliage Here' section as seen in 3.a.
    - If the user doesn't already have a foliage actor, click and drag your static mesh into the 'Drop Foliage Here' section and Unreal will create one for you. (4.a)
  <div class="img_container">
      <img class='sm_img' src="../imgs/UnrealPic_23.png"/>
  </div>
1. Now hover the mouse over the new foliage actor and check the empty box and select the tree.
1. To find a short video on this subject [click here](https://monodrive.readthedocs.io/en/latest/unreal_tutorials/FoliagePainting/) to go to the monoDrive read the docs section about it.

### Adjusting the Foliage Settings

A good starting point is something like this (5.a):
  <div class="img_container">
      <img class='md_img' src="../imgs/UnrealPic_24.png"/>
  </div>

### Setting Collision for Grasses and Bushes

1. Foliage actors such as small bushes, grasses and other small foliage's collision setting should look as such (6.a) :
  <div class="img_container">
      <img class='md_img' src="../imgs/UnrealPic_25.png"/>
  </div>(6.a)

1. Foliage actors such as large bushes, boulders and trees should have their collision setting set as such (7.a):
  <div class="img_container">
      <img class='md_img' src="../imgs/UnrealPic_26.png"/>
  </div>(7.a)

### Setting Collision for Trees

1. Navigate to the static mesh of the tree
    - Scroll down the 'Details' panel until 'Collision' section is found
    - Find the option 'Collision Complexity'
    - Set the 'Collision Complexity' to 'Simple and Complex'
1. Next, change the 'Collision Presets' to 'Custom'
    - Set 'Radar' to 'Ignore'
    - Set all the other settings to 'Block'
1. If everything is set properly it should look like this (8.a) :
  <div class="img_container">
      <img class='wide_lg_img' src="../imgs/UnrealPic_27.png"/>
  </div>(8.a)

### Setting the Physical Materials for Foliage

1. Navigate to the materials that the tree is using
    - Open the various materials
1. In the materials 'Details' panel navigate to the 'Physics Materials' section
1. Click the drop down menu
    - Select the 'Tree' physical material or the physical material that matches best to the object. (9.a)
  <div class="img_container">
      <img class='sm_img' src="../imgs/UnrealPic_28.png"/>
  </div>(9.a)

#### Painting Trees

To paint the trees set up the brush settings. 

1. Start with a low 'Paint Density' setting, this keeps the scene from being overloaded. 
1. Under the 'Filters' section have only the kinds of objects that want to be painting on selected. For example: Landscape or Static Meshes.
1. Now with everything set move your mouse into the viewport, there should be a semi-transparent grey dome.
1. 'Left' click in the viewport, the user should see trees start to pop up.
    - If need be, adjust the settings that were talked about above to adjust how many trees are being painted.
    - To erase trees hold down 'shift' and left click and hold over the trees to erase.