# Bounding Box on a Skeletal Mesh Actor

This section will explain how to add custom bounding boxes to skeletal mesh actors 
in the scene. 

These custom bounding boxes will provide 2D regions of interest and 3D Object Oriented bounding boxes 
in the annotation data of various sensors like the Camera. 
In this example, the bounding box on a vehicle’s rear windshield will be added and displayed when using a camera sensor.
This is useful when the user would like to get ground truth information about some area of a visible actor. 

## Add a Custom Bounding Box to a Vehicle

In this tutorial, a custom bounding box is added to `sedan_monoDrive_01` on the rear windshield. 

1. Navigate to `Vehicles` under `Content Browser` in the Scenario Editor.


2. Right click on the `sedan_monoDrive_01` vehicle and click on ‘Edit’ as shown below
    <p class="img_container">
    <img class="wide_img" src="../img/sklmesh_step2.png"/>
    </p>
    
3. Double click on the Skeletal Mesh icon
    <p class="img_container">
    <img class="wide_img" src="../img/sklmesh_step3.png"/>
    </p>

4. Click on Physics and the result will look like the image below.
    <p class="img_container">
    <img class="wide_img" src="../img/sklmesh_step4.png"/>
    </p>

5. Right click on vehicle_root and select Add Shape &rarr; Add Box 
   (The box added is highlighted in the image below)
    <p class="img_container">
    <img class="wide_img" src="../img/sklmesh_step5.png"/>
    </p>

6. Place cursor around the area the box was added and zoom in (using mouse wheel scroll up direction)
   to the vehicle till the newly added box is visible.

    A.	On the top of the window, change the tool to `Select and Translate Objects`

    B.  Click on the box shape added – the box should be movable in `x`, `y` and `z` directions. 
        <p class="img_container">
        <img class="wide_img" src="../img/sklmesh_step6.png"/>
        </p>

7.	Use the moving tools to place it on center of rear windshield of the vehicle

    A.	Zoom out of the vehicle using the mouse wheel’s scroll down gesture.
        <p class="img_container">
        <img class="wide_img" src="../img/sklmesh_step7a.png"/>
        </p>

    B.	Click on Perspective &rarr; then click on `Back` to change to the rear view of the vehicle. 
        <p class="img_container">
        <img class="wide_img" src="../img/sklmesh_step7b.png"/>
        </p>

    C. Locate the box and align it to the rear windshield of the vehicle as close as possible 
        (still   using the `Select and Translate Objects` tool)
        <p class="img_container">
        <img class="wide_img" src="../img/sklmesh_step7c.png"/>
        </p>

    D.	Click on ‘Back’ and change the view to ‘Left’ or ‘Right’. 

    E.	Click on the `Select and Rotate objects` tool on the top right corner of the window. Rotate the windshield box to align with the angle of the windshield
        <p class="img_container">
        <img class="wide_img" src="../img/sklmesh_step7e.png"/>
        </p>

    F.	Change to ‘Back’ view of the vehicle. Use the `Select and Scale objects` tool to scale the size of the added box to fit the rear windshield (use the mouse to drag in the upward and rightward directions).
        <p class="img_container">
        <img class="wide_img" src="../img/sklmesh_step7f.png"/>
        </p>

    G.	Keep switching between the Perspective, Left and Back views until the box looks aligned with the rear windshield. 
        <p class="img_container">
        <img class="wide_img" src="../img/sklmesh_step7g.png"/>
        </p>

8.	Click on ‘Save’ on the top left corner. 

9.	With the newly added box still selected, look for
Details&rarr;Body Setup&rarr;Primitives&rarr;Boxes&rarr;Name.
Enter the name ‘rear_windshield’

10.	Uncheck the ‘Contribute to Mass’ parameter so the box only   serves annotation purposes. This disables the overall mass of the added shape. 

11.	Click on ‘Save’ on the top left corner. 

## Setting up the C++ client

This section of the document explains how to modify code on the camera_sensor.cpp client example to specifically extract the above modified 2D bounding box with the label ‘rear_windshield’ using annotations

<div style="display: inline-block;">
1. Navigate to monodrive-client&rarr;examples&rarr;cpp&rarr;dev&rarr;camera_sensor.cpp file. 
Make the following change to include the vehicle’s actor tag and save the file.
</div>
```cpp
    fc_config.annotation.desired_tags = {"vehicle"};
```
<div style="display: inline-block;">
2. Make the following changes and save the file, to only view the rear windshield’s bounding box.
</div>
```cpp
    for (auto &bbox : annotation.second.bounding_boxes_2d)
               if(bbox.name == "rear_windshield") {
                cv::rectangle(img, cv::Point(int(bbox.xmin), int(bbox.ymin)),
                              cv::Point(int(bbox.xmax), int(bbox.ymax)),  
                              cv::Scalar(0, 0, 255));
               }
```

<div style="white-space: pre-wrap;">
3. 	Build the client C++ file. 

4.	Press ‘Play’ on the monoDrive Scenario Editor.

5.	Debug & Run ‘camera_sensor.cpp’. The bounding boxes for the desired actors will be visible in the opencv window.
</div>