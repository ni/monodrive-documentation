# Placing Road signs

1. All of the signs in the Content → Meshes → Signs Folder are mesh templates
    - All of the sign materials in the Content → Materials → Signs Folder will match with their corresponding names.
1. Click and Drag the the sign you would like to use from the ‘Content’ browser into the viewport, ex: Diamond_36_Sign
    - In the details panel → Materials there will be an empty ‘Element’ slot

<div class="img_container">
<img class='lg_img' src="../imgs/road_signs.png"/>
</div>

1. Navigate to the Content → Materials → Signs Folder, pick the appropriate sign shape folder, ex: Diamond_Signs, and open the folder
1. Find the material that sign is needed.
1. Click and drag the material in to the empty ‘Element slot’ 

# Placing Mesh Splines

1. Navigate to the Content → Bluprints 
    - Find the blueprint called ‘Spline_Actor’
1. Drag said blueprint into the viewport
    - This will already have the ‘concrete barrier’  asset assigned to the Mesh slot (1.b)

<div class="img_container">
<img class='lg_img' src="../imgs/mesh_spline.png"/>
</div>

There will be two spline points near the ‘concrete barrier’ 
Click and drag the spline point the ‘Translate’, ‘Rotate’ or ‘Scale tool is not over. (1.c)
(1.c)
Now replace the ‘concrete barrier’ asset with a static mesh of your choosing
The Mesh slot is located in the Detail Panel
Side note: you can use flowing assets like the concrete barrier/guardrails or you can place repeatable individual assets like the street lights/Light posts.

Now adjust the ‘Spacing’ value in the details panel
Side note depending on whether you are adjusting a flowing asset or a repeatable asset, the ‘Spacing’ value can vary greatly.
Rule of Thumb: the larger the ‘Spacing value the more space apart the asset will be
A good starting point for ‘flowing’ assets is: 200-600
A good starting point for repeatable assets is: 2000-5000
Spline Curving
When needing to curve the spline asset do the following
For bastic curves, double click on either of the 2 original spline points with your rotation tool selected
You will see two ‘arms’ display themselves, you will now be in the proper mode
Now use the rotate tool to curve the spline (2.a)
(2.a)
For more complex curves, right click on the spline itself and click ‘Add Spline Point Here’
As before, double click on the spline point
Use the ‘Translate’ and ‘Rotate tool to adjust the spline into the position you require. 



Painting Trees
Setting Your Foliage Up
To paint foliage into your scene you must go to the ‘Foliage’ mode located in the top left corner of Unreal. (Note this is true for all Unreals up to 4.24) (3.a)
(3.a)

To add foliage to the ‘Foliage’ Painter tool, navigate to where your foliage content is. Ex: Content → Foliage → Trees → RedMaple
If you already have a foliage actor, indicated by the green line at the bottom of the asset drag it into the ‘Drop Foliage Here’ section as seen in 3.a.
If you don’t already have a foliage actor, click and drag your static mesh into the ‘Drop Foliage Here’ section and Unreal will create one for you.

Now hover your mouse over the new foliage actor and check the empty box and select the tree.
Adjusting the Settings
A good starting point is something like this: 


Painting Your Trees
To paint your trees first you must set up your brush settings:
So to not overload your scene with too many trees start with a low ‘Paint Density’ setting, For example: .1
Under the ‘Filters’ section have only the kinds of objects that you want to be painting on selected. For example: Landscape or Static Meshes
Now that you have everything set move your mouse into the viewport, you should see a semi-transparent blue dome
Now ‘Left’ click in the viewport, you should see trees start to pop up.
If need be, adjust the settings that were talked about above to adjust how many trees are being painted.

