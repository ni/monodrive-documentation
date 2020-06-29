 
# New Vehicle Model 

The monoDrive Scenario Editor provides users with the ability to import and export new vehicle models for more realistic simulation. When adding a new vehicle, reducing the size/ poly count of that asset will smooth the experience while simulating.

## Reduction

3D modeling software programs enable users to reduce the size of a model for faster and smoother simulation process. Resources for some of these programs: 

[Blender Asset Reduction](somelink.com)

[Maybe another program](somelink.com)

### Model Reduction from Maya

1. How does one check that the various parts of the model are combined into groups and/or add the parts to groups. I.e. Truck bed & Truck cab?
1. Set the origin points to the origin of the world (0,0,0).
1. Export the model as FBX to UE4.
    - Check that parts are not too small or have single polys, this could result in a less optimal result.

### UE4 Reduction

 1. Import all of the various parts as static meshes. (picture of where this is menu is in UE4)
 
 
 
 1. Reduce a single mesh
    1. Double click on a mesh to open.
    1. On the right, under LOD settings, find the setting "Number of LODs."
    1. Move the slider to the 2 position and click "apply changes."
        - In the top left, there is a rectangle that says LOD Auto, click on it and change it to LOD 1
        - the new model should be viewable and the reduction settings should be adjustable.
    1. On the Right, open the menu "LOD 1" and click "Reduction Settings" (If not found, use the search bar at the top)
        - Adjust the "Percent Triangles" slider to adjust the current LOD of the model.
        - Click apply to see the results. 

Once all static meshes have been reduced, save the reduced asset. (picture of this as well.)

 1. Drag a vehicle asset into the viewport
 1. Right click on the asset, click on "Merge Actors"
    - Open options and click on "Use specific LOD level" (Why?)
    - Change the Specific LOD option from 0 to 1
    - Click "Merge Actors"
    - Select a location within UE4 to save the reduced asset


### UE4 Reduction Export

 1. In the folder location where the exported asset(s) have been saved.
    - Right click on the asset
    - Go to "Asset Actions" → "Export" 
    - Select a location to save the reduced model.
    - These assets can be imported into any modeling program. I.e. Maya, Blender

## Export from Maya

Before exporting:

1. Combine all of the car parts into a single body except the tires.
1. Name the body vehicle_root, and name the tires accordingly and as follows:
    - FR_wheel
    - FL_wheel
    - BL_wheel
    - BR_wheel

    <div class="img_container">
      <img class='sm_img' src="../imgs/vehicle_read.png"/>
    </div>

1. Rotate the body of the vehicle's "X" position by 90 degrees or till 0 degrees on the "X" rotational axis. 
- could we just say rotate to -90 degrees?
1. Rotate it back to its original position so that the "X" rotational axis says "-90", the body is prepped.

1. To connect the tires to the body of the car and force the axises to mimic the axis of the vehicle:
    - open "Outliner" from the (what menu?). 
    - select all of the properly named tires and middle mouse drag them onto the “vehicle_root” (car body) (Could we add a description for right clicking too?, or a menu click).
1. Make sure that the pivot point for the vehicle is at the origin point of the world, (0,0,0). (how do I check this?)
1. Make sure to freeze the rotation and scale for the car and wheels of the vehicle. (what? how?)

    <div class="img_container">
      <img class='wide_img' src="../imgs/truck_model.png"/>
    </div>

Exporting from Maya:

1. Select the vehicle, go to "File" ----> "Export selection"
1. Select a location to save the asset
1. Set file type to FBX .
1. Click export.

## Import

1. To import a model into UE4
    - Drag and drop the FBX file, or
    - Click the green "import" button and navigate to the FBX File
1. After the FBX Import Options open: ( a picture of this would be really helpful, with the options highlighted)
    - Check the "Skeletal Mesh"
    - Expand the "Mesh category"
        - Set "Normal Import Method" to "Import Normals and Tangents".
1. Click "Import All"


## Vehicle Settings

Are these mandatory set-up instructions? What is the purpose of the vehicle skeleton?

### Vehicle Skeleton

1. Open the Vehicles Skeleton (picture of the below parameters?)
    - the "root" should be in the center beneath the vehicle
    - each wheel should have a yellow "Bone" going to it
1. Open the Physics mesh and change the "Roots Collision" from "pill" shape to "box" shape. (does this make sense still?)
1. Adjust the new box in the editor as close to the car as possible.
1. Slim down the wheels collision to match the diameter of the tire. (what do you mean by slim down? like the size?)
1. Go to the Skeletal mesh and replace or adjust textures as needed. 
    - NOTE: Double check that each material has the “Physical Material” that matches it best before moving on from this step.
1. *Change the names to Body, FL_wheel, FR_wheel, BL_wheel, BR_wheel

### Wheel Class

No instructions here.

1. 


### Connecting to UE4 Blueprint
1. In the Scenario Editor under "Content", open the "Vehicles" folder.
1. Create a child of the parent vehicle blueprint ( It should be the one that does not look like a car.) (how?)
1. Open it up and click on the Mesh (Inherited) option in the top left of the blue print.
    - At the right of the window, place "Skeletal Mesh" in the slot (could we add another picture here of this)
1. Set up the HD Static mesh
    - Go to the "Vehicles" Folder in the "Content" folder and drag the skeletal mesh vehicle into the viewport. 
    - Right click the "Skeletal Mesh" ---> "Convert 'skeletal mesh name'” to Static mesh".
    - Save the new static mesh in the same location as the vehicle's assets.
1. Open the blueprint
1. In upper left corner, click "HDMesh (Inherited)" option
1. On the right there should be an option to put the HD Static Mesh you just created. Open it up and locate it. (what is the users objective here?)

### Creating the Animation Blueprint
1. In the Scenario Editor under "Content", open the "Vehicles" folder.
1. Select the vehicle
1. Right click, select “Animation” ---> "Animation Blueprint"
1. In the "Parent Class Type" in the search bar: VehicleAnimInstance and select it (not following exactly)
1. In the "Target Skeleton", type in the search bar the name of the vehicle's skeleton and select it.
1. Click "Okay" (or "ok")
1. Name Anim file
1. Find and connect the "Mesh Space Ref Pose", "Wheel Handler", and "Component To Local" to the "Output Pose" in the Anim Blueprint.

    <div class="img_container">
      <img class='wide_img' src="../imgs/anim_blueprint.png"/>
    </div>

1. Open up the Blueprint 
    - Find the section of the Blueprint that says Anim
    - Attach the Anim file created here.
1. Go back to the "Vehicles" folder in the content browser, drag the vehicle blueprint into the viewport on to a road spline.
1. Click play to see the vehicle's behavior in the simulation.