
# monoDrive: RoadRunner to Unreal Pipeline 2

### Overview

These walkthroughs go through each of the monoDrive methods of bringing the maps created in RoadRunner into Unreal and to get the
 For more information, see [monodrive Documentation](https://monodrive.readthedocs.io/en/latest/).

For [Unreal Engine Tutorials](https://monodrive.readthedocs.io/en/latest/unreal_tutorials/MayaWheels/) for guides.

For [Maya Tutorial](https://knowledge.autodesk.com/search-result/caas/simplecontent/content/maya-1-minute-startup-movies.html) for guides.


#### Importing RoadRunner Exports

Once you have exported your assets from RoadRunner, you are now ready to import those assets into Unreal.

NOTE: If you are just trying to setup a map this walkthrough will still be useful.

- First Open Unreal
  - With Unreal open create a new map

<div class="img_container">
    <img class='lg_img' src="../imgs/UnrealPic_01.png"/>
</div>

  - Then click on 'Default'
- Once you've created your new map go to the 'Landscape' folder in the 'Content' browser

<div class="img_container">
    <img class='lg_img' src="../imgs/UnrealPic_02.png"/>
</div>

- Then click on 'Import' and navigate to your RoadRunner .fbx export and open
- A 'FBX Import Option' window will open
  - Find the 'Import Uniform Scale' option
    - Type in 100.0
- Click 'Import All'
- Click and Drag all assets into the scene
- Now with all assets selected go to the 'Details' Panel
  - In the 'Location' section in the X,Y,Z section type in a 0 for each slot

<div class="img_container">
    <img class='lg_img' src="../imgs/UnrealPic_03.png"/>
</div>

#### Setting up RoadRunner Asset Collision

To make your road drive able you need to adjust the collision of the assets you've imported.

- First open any of the assets that you imported by double clicking on them
- Scroll down or search the 'Details' panel for the 'Collision' section
  - Now adjust the 'Collision Complexity' so that it reads 'Use Complex Collision as Simple'
  - Then adjust the 'Collision Presets' so that it reads 'Road' or 'Landscape'

<div class="img_container">
    <img class='lg_img' src="../imgs/UnrealPic_04.png"/>
</div>

- Repeat on your other imported meshes

#### Adjusting Imported Materials

- Now to make your materials viewable to radar you must attach 'Physical Materials'.
- Open any of your materials
- Find the 'Details' panel on the left side of the screen
  - Find where it says 'Phys Material'
  - Click on the drop down where it says 'None'
  - Apply the material that applies most to your material type

<div class="img_container">
    <img class='lg_img' src="../imgs/UnrealPic_05.png"/>
</div>

#### Importing OpenDrive

- With your road and landscape imported it is time to Import your OpenDrive file.

- If you do not see a tab open in Unreal labled 'RoadNetworkToolWidget' go to the top level of the 'Content' structure

- Next right click on the icon that says 'RoadNetworkToolWidget'

- Now click on 'Run Editor Utility Widget'

<div class="img_container">
    <img class='lg_img' src="../imgs/UnrealPic_06.png"/>
</div>

- You should see a window that looks like this:

<div class="img_container">
    <img class='lg_img' src="../imgs/UnrealPic_07.png"/>
</div>

- Click on the 'Load' option

- Find your exported Opendrive file from RoadRunner

- Now go to the 'World Outliner'

- Find the asset named 'OpenDriveRoadNetwork' and click on it

- Now navigate to the 'Details' panel

- In the 'OpenDrive' section you should see a button labled 'Generate OpenDRIVE Mesh'

- Click on it.

- NOTE: all this is doing is confirming that your OpenDrive and your road are matching up

- If everything is done correctly, it should look something like this:

<div class="img_container">
    <img class='lg_img' src="../imgs/UnrealPic_08.png"/>
</div>

- To get rid of the mesh click on the 'Clear OpenDRIVE Mesh'

#### Weather

To quickly apply weather to your new level follow these steps.

- First go to the top left of Unreal and find 'Window' → 'Level's

<div class="img_container">
    <img class='lg_img' src="../imgs/UnrealPic_09.png"/>
</div>

- Next go to the 'Content' Browser to the 'Maps' → 'SubLevels' folder
  - Make a folder with your maps name as the title

<div class="img_container">
    <img class='lg_img' src="../imgs/UnrealPic_10.png"/>
</div>

  - Next, open the 'Straightaway5k' folder
  - Copy the 'straightaway5k\_Weather' sublevel to your levels sublevel folder
  - Next rename the file to "YourLevelNameHere\_Weather"

<div class="img_container">
    <img class='lg_img' src="../imgs/UnrealPic_11.png"/>
</div>

- Once this is done, click and drag the sublevel over to the 'Levels' tab you opened earlier

<div class="img_container">
    <img class='lg_img' src="../imgs/UnrealPic_12.png"/>
</div>

- Right click on the sublevel and go to 'Change Streaming Method' → 'Always Loaded'

<div class="img_container">
    <img class='lg_img' src="../imgs/UnrealPic_13.png"/>
</div>

- Next go back to the 'World Outliner'
  - Delete anything that isn't in the 'Atmosphere' folder and that isn't your Imported assets.
  - It should look like this:

<div class="img_container">
    <img class='lg_img' src="../imgs/UnrealPic_14.png"/>
</div>

#### Static Assets

Now it is time to populate your environment.

#### Placing Road Signs

- All of the signs in the Content → Meshes → Signs Folder are mesh templates

- All of the sign materials in the Content → Materials → Signs Folder will match with their corresponding names.

- Click and Drag the the sign you would like to use from the 'Content' browser into the viewport, ex: Diamond\_36\_Sign

- In the details panel → Materials there will be an empty 'Element' slot (1.a)

<div class="img_container">
    <img class='md_img' src="../imgs/UnrealPic_15.png"/>
</div>1.a

- Navigate to the Content → Materials → Signs Folder, pick the appropriate sign shape folder, ex: Diamond\_Signs, and open the folder

- Find the material that sign is needed.

- Click and drag the material in to the empty 'Element slot'

- To find a short video on this subject [click here](https://monodrive.readthedocs.io/en/latest/unreal_tutorials/PlacingSigns/) to go to the monoDrive read the docs section about it.

## Setting Custom Depth Stencil Values

#### Overview

Use this tutorial to set up the collision

#### Application:

- As stated above, place a static mesh into your scene
- Now with the mesh selected, locate the Details panel
  - Type 'custom' into the search bar (2.a)
- Check the box labeled 'Render CustomDepth Pass'
- Go to the link provided and fill in the Custom Depth Stencil Value that matches your static mesh the closest.
  - [Data Products — monoDrive Simulator](https://monodrive.readthedocs.io/en/latest/r2v/data_processing_output/)

<div class="img_container">
    <img class='lg_img' src="../imgs/UnrealPic_16.png"/>
</div>(2.a)

## Setting up Collision for Static Meshes

#### Overview

Use this tutorial to set up the collision

#### Application:

- Open up your static mesh
  - In the middle of the screen you should see the 'Collision' option

<div class="img_container">
    <img class='sm_img' src="../imgs/UnrealPic_17.png"/>
</div>

- There should be a green box around your static mesh
  - If not, look in the top right of your screen and find the 'Collision' drop down menu
  - You can try any of the collision options, but a good starter is the box collision
- Now in the 'Details' panel on the right
  - Search collision
  - You will want to set the 'Collision Presets'
  - Select the preset that match's your object the best
  - For most static meshes the 'BlockAll' option is best
  - Now in the 'Details' panel on the right
  - For any landscape static meshes select the 'Landscape' option


Spline Assets

#### Placing Mesh Splines

- Now in the 'Details' panel on the right
  - Find the blueprint called 'Spline\_Actor'
- Drag said blueprint into the viewport
  - This will already have the 'concrete barrier' asset assigned to the Mesh slot (1.b)

<div class="img_container">
    <img class='md_img' src="../imgs/UnrealPic_18.png"/>
</div>

- There will be two spline points near the 'concrete barrier'
  - Click and drag the spline point the 'Translate', 'Rotate' or 'Scale tool is not over. (1.c)

<div class="img_container">
    <img class='lg_img' src="../imgs/UnrealPic_19.png"/>
</div>

- Now replace the 'concrete barrier' asset with a static mesh of your choosing
  - The Mesh slot is located in the Detail Panel
  - Side note: you can use flowing assets like the concrete barrier/guardrails or you can place repeatable individual assets like the street lights/Light posts.

<div class="img_container">
    <img class='lg_img' src="../imgs/UnrealPic_20.png"/>
</div>

- Now adjust the 'Spacing' value in the details panel
  - Side note depending on whether you are adjusting a flowing asset or a repeatable asset, the 'Spacing' value can vary greatly.
  - Rule of Thumb: the larger the 'Spacing value the more space apart the asset will be
- A good starting point for 'flowing' assets is: 200-600
- A good starting point for repeatable assets is: 2000-5000
- To find a short video on this subject [click here](https://monodrive.readthedocs.io/en/latest/unreal_tutorials/SingularMesh/) to go to the monoDrive read the docs section about it.

#### Spline Curving

- When needing to curve the spline asset do the following
  - For basic curves, double click on either of the 2 original spline points with your rotation tool selected
    - You will see two 'arms' display themselves, you will now be in the proper mode
  - Now use the rotate tool to curve the spline (2.a)

<div class="img_container">
    <img class='lg_img' src="../imgs/UnrealPic_21.png"/>
</div>

(2.a)

  - For more complex curves, right click on the spline itself and click 'Add Spline Point Here'
  - As before, double click on the spline point
    - Use the 'Translate' and 'Rotate tool to adjust the spline into the position you require.
- To find a short video on this subject [click here](https://monodrive.readthedocs.io/en/latest/unreal_tutorials/SplineMesh/#adjusting-a-spline-mesh) to go to the monoDrive read the docs section about it.

Foliage Assets

#### Painting Trees

#### Setting your Foliage Up

- To paint foliage into your scene you must go to the 'Foliage' mode located in the top left corner of Unreal. (Note this is true for all Unreals up to 4.24) (3.a)

<div class="img_container">
    <img class='sm_img' src="../imgs/UnrealPic_22.png"/>
</div>(3.a)

- To add foliage to the 'Foliage' Painter tool, navigate to where your foliage content is. Ex: Content → Foliage → Trees → RedMaple
- If you already have a foliage actor, indicated by the green line at the bottom of the asset drag it into the 'Drop Foliage Here' section as seen in 3.a.
- If you don't already have a foliage actor, click and drag your static mesh into the 'Drop Foliage Here' section and Unreal will create one for you. (4.a)

<div class="img_container">
    <img class='sm_img' src="../imgs/UnrealPic_23.png"/>
</div>

- Now hover your mouse over the new foliage actor and check the empty box and select the tree.
- To find a short video on this subject [click here](https://monodrive.readthedocs.io/en/latest/unreal_tutorials/FoliagePainting/) to go to the monoDrive read the docs section about it.

#### Adjusting the Foliage Settings

A good starting point is something like this (5.a):

<div class="img_container">
    <img class='md_img' src="../imgs/UnrealPic_24.png"/>
</div>

_Setting Collision for Trees Pt. 1__ _

Foliage actors such as small bushes, grasses and other small foliage's collision setting should look as so (6.a) :

<div class="img_container">
    <img class='md_img' src="../imgs/UnrealPic_25.png"/>
</div>(6.a)

Foliage actors such as large bushes, boulders and trees should have their collision setting set as so (7.a):

<div class="img_container">
    <img class='md_img' src="../imgs/UnrealPic_26.png"/>
</div>(7.a)

#### Setting Collision for Trees Pt. 2

- Navigate to the static mesh of your tree
  - Scroll down the 'Details' panel until you find the 'Collision' section
  - Find the option 'Collision Complexity'
  - Set the 'Collision Complexity' to 'Simple and Complex'
- Next, change your 'Collision Presets' to 'Custom'
  - Set 'Radar' to 'Ignore'
  - Set all the other settings to 'Block'
- If everything is set properly it should look like this (8.a) :

<div class="img_container">
    <img class='wide_lg_img' src="../imgs/UnrealPic_27.png"/>
</div>(8.a)

#### Setting your Physical Materials for Foliage

- Navigate to the materials that your tree is using
  - Open the various materials
- In the materials 'Details' panel navigate to the 'Physics Materials' section
- Click the drop down menu
  - Select the 'Tree' physical material or the physical material that matches best to your object. (9.a)

<div class="img_container">
    <img class='sm_img' src="../imgs/UnrealPic_28.png"/>
</div>(9.a)

#### Painting your Trees

To paint your trees first you must set up your brush settings:

- So to not overload your scene with too many trees start with a low 'Paint Density' setting, For example: .1
- Under the 'Filters' section have only the kinds of objects that you want to be painting on selected. For example: Landscape or Static Meshes.
- Now that you have everything set move your mouse into the viewport, you should see a semi-transparent grey dome.
- Now 'Left' click in the viewport, you should see trees start to pop up.
  - If need be, adjust the settings that were talked about above to adjust how many trees are being painted.
  - To erase trees hold down 'shift' and left click and hold over the trees you want to erase.

INTERNAL - NI CONFIDENTIAL

ni.com