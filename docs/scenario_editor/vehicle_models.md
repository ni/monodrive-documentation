 
# New Vehicle Model 

## Quick Model Reduction Maya to UE4

### Maya before you Export

1. Check that the various parts of the model are combined into groups and/or add the parts to groups. I.e. Truck bed & Truck cab.
1. Once the model has been placed in various groups, check the origin points are at the origin of the world (0,0,0).
1. Export the models as FBX to UE4.
    *Note* Check that parts are not too small or have single polys, this could result in a less optimal result.

### UE4 Reduction

 1. Import all of the various parts as static meshes.
 1. Double click and open up the first mesh.
 1. On the right, under LOD settings, find the setting "Number of LODs."
 1. Move the slider to the 2 position and click "apply changes."
    - In the top left, there is a rectangle that says LOD Auto, click on it and change it to LOD 1
 1. Now the new model will be viewable and the reduction settings should be adjustable.
 1. On the Right, open the menu "LOD 1" and click "Reduction Settings" (If not found, use the search bar at the top)
    - Adjust the ‘Percent Triangles’ slider to adjust the current LOD of the model.
    - Click apply to see the results. 
 1. Repeat if necessary 

### UE4 Export
 1. Drag your first vehicle asset into the viewport
 1. Right click on the asset, click on ‘Merge Actors’
    - Find LODSelection Type
    - Open its options and click on Use specific LOD level
    - Change the Specific LOD option from 0 to 1
    - Click ‘Merge Actors’ 
    - Select a location within UE4 to save your reduced asset
    - Rinse and repeat for any assets you may have left
    - Now that you have all of your assets reduced and saved it is time to export.
 1. In the folder where you exported your asset(s) do the following:
    - Right click on your asset
    - Go to ‘Asset Actions’ → ‘Export’ 
    - Now Select the Folder/Location that you would like to put your newly reduced model into.
    - Rinse and repeat for any left over assets
    - Now feel free to import these assets into any modeling program. I.e. Maya, Blender

## Maya

### Before you Export

1. Combine all of the car parts into a single body, leave the tires as separate objects. 
1. Name the body vehicle_root, and name the tires accordingly and as follows:
    - FR_wheel
    - FL_wheel
    - BL_wheel
    - BR_wheel
    - Now that everything is named properly you need to set the model up so that Unreal will read the model properly. If you have everything set-up properly it should look something like this(Note:Already parented per Step 5): 

    <div class="img_container">
      <img class='sm_img' src="../imgs/vehicle_read.png"/>
    </div>

1. Now rotate the body of the vehicle’s ‘X’ by 90 and freeze its position so it says 0 on its ‘X’ rotational axis. 
1. Now rotate it back to its original position so that its ‘X’ rotational axis says ‘-90’, the body is now prepped.
1. Now open the outliner, and select all of the properly named tires and middle mouse drag them onto the “vehicle_root” (car body), this parents the tires to the body of the car and forces their axis to mimic the axis of the vehicle.
1. Make sure that the pivot point for the vehicle is at the origin point of the world. (0,0,0)
1. Make sure to freeze the rotation and scale for the car and wheels of the vehicle.
1. If everything has been  done correctly it should look like this:

    <div class="img_container">
      <img class='wide_img' src="../imgs/truck_model.png"/>
    </div>

### Exporting

1. Now you are ready for export.
1. Select the vehicle, go to file ----> export selection
1. Set up a place for your file to be placed.
1. The file type should be set to FBX (if it is not UE4 will not read it properly)
1. Click export.

## Unreal 4

### Importing

1. To import a model into UE4 you can either:
    - Drag and drop the FBX file from its location in Windows or,
    - Click the green import button and navigate to where your FBX file is
1. Now the FBX Import Options should pop up, you now need to:
    - Check the ‘Skeletal Mesh’ check box
    - Expand the ‘Mesh category’ and make sure that the ‘Normal Import Method’ is set to: Import Normals and Tangents.
1. Now click ‘Import All’

## Setting Your Vehicle up

### Vehicle Skeleton
1. Open your Vehicles Skeleton, the ‘root’ should be in the center beneath the vehicle, and each wheel should have a yellow ‘Bone’ going to it
1. Now go to your Physics mesh and change the Roots Collision from a ‘pill’ shape to a box. 
1. Adjust the new box to be as close to the car as possible.
1. Next slim down the wheels collision to match the diameter of the tire.
1. Now go to the Skeletal mesh and replace or adjust textures as needed. 
    - NOTE: Double check that each material has the “Physical Material” that matches it best before moving on from this step.
1. *Change the names to Body, FL_wheel, FR_wheel, BL_wheel, BR_wheel

## Setting Up the Wheel Class

1. 


## Setting Connecting to the UE4 Blueprint
1. Go to the Vehicles folder
1. Create a child of the parent vehicle blueprint ( It should be the one that does not look like a car.)
1. Open it up and click on the Mesh (Inherited) option in the top left of the blue print.
    - Now look to the right of the window you should see a slot in which to put the ‘Skeletal Mesh’
1. Now you must set up the HD Static mesh.
    - Go back to the ‘Vehicles’ Folder in the Content folder and drag the skeletal mesh vehicle you are currently working on, into the viewport. 
    - Right click on said ‘Skeletal Mesh’  and look for the option to ‘convert “skeletal mesh name here” to Static mesh’.
    - Save the new static mesh where you are storing the rest of the vehicle's assets.
1. Now open the blueprint back up and look back to the upper left and click on the HDMesh (Inherited) option and click on it.
1. Now on the right there should be an option to put the HD Static Mesh you just created. Open it up and locate it.

### Creating the Animation Blueprint
1. First go to the folder of your vehicle
1. Right click and find “Animation”
1. Options should present themselves, find Animation Blueprint and click on it
1. In the Parent Class type in the search bar: VehicleAnimInstance and select it
1. In the Target Skeleton, type in the search bar the name of your vehicle's skeleton and select it.
1. Click ‘Okay’
1. Name Anim file appropriately
1. Now Find and hook up, Mesh Space Ref Pose, Wheel Handler and Component To Local to the Output Pose in the Anim Blueprint. As seen below:

    <div class="img_container">
      <img class='wide_img' src="../imgs/anim_blueprint.png"/>
    </div>

### Finishing up
1. Now open up your Blueprint you should see a section of your Blueprint that says Anim., attach the Anim file you just created here.
1. Go back to the Vehicles folder in the content browser, drag your vehicle blueprint out into the viewport on to a road spline.
1. Click play and hopefully if you’ve followed everything properly your car should drive!