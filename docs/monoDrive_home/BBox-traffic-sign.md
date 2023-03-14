# Bounding Box on a Static Mesh Actor

This section will explain how to add custom bounding boxes to static mesh actors in your scene. 

These custom bounding boxes will provide 2D regions of interest in the annotation data 
of various sensors like the Camera. 
In this case, the bounding box on a traffic sign and streetlight will be modified and displayed when using 
a camera sensor.
This is useful when the user would like to get ground truth information about some area of a 
visible actor. 

For example, the user may want to get a bounding box just for the face of a traffic sign for classification purposes and would otherwise like to ignore the other parts like the shaft of the sign.


## Display Only the Sign's Bounding Box of a Traffic Sign

The goal of this section is to use the existing bounding boxes on a traffic sign’s static mesh 
and display only the desired region of interest 
(For example, the actual sign and not the shaft)

1. Open the `Content Browser` on the monoDrive Scenario Editor. 

2. Navigate to `Content`&rarr;`Maps`
    
3. Open the `Straightaway5k` map. The purpose of this step is to find the sign and its static mesh from the map.
    <p class="img_container">
    <img class="wide_img" src="../img/stmesh_traffic_step3.png"/>
    </p>

4. In the World Outliner, look for ‘Rectangle_Vertical’. 
    
5. Select one of them, and open the Static Mesh by double-clicking, from the 'Details' window as seen below. Please note that all the 'Rectangular_Vertical signs' will be updated when modifying the static mesh of ‘Rectangle_Vertical’ signs.
    <p class="img_container">
    <img class="wide_img" src="../img/stmesh_traffic_step5.png"/>
    </p>

6. In the Static Mesh window of this asset, click on Collison &rarr; Check the 'simple collision' box
    <p class="img_container">
    <img class="wide_img" src="../img/stmesh_traffic_step6.png"/>
    </p>

7.	The bounding boxes of the current asset will be visible
    <p class="img_container">
    <img class="wide_img" src="../img/stmesh_traffic_step7.png"/>
    </p>
 
8.	Under Details&rarr;Collision&rarr;Primitives&rarr;Boxes, the existing bounding boxes are visible along with their names.
The actual sign is the 'face', and the pole is the 'shaft'

9.	Uncheck the ‘Contribute to Mass’ parameter so the box only   serves annotation purposes. 
This disables the overall mass of the added shape. 

10.	Click on ‘Save’ on the top left corner. 

## Setting up the C++ client for a Traffic Sign

This section of the document explains how to modify code on the camera_sensor.cpp client example to specifically extract only the sign’s 2D bounding box with the label ‘face’ using annotations. 

<div style="display: inline-block;"> 1. Navigate to monodrive-client&rarr;examples&rarr;cpp&rarr;dev&rarr;camera_sensor.cpp file. 
    Make the following change to include the traffic sign’s actor tag and save the file: 
</div>
```cpp
    fc_config.annotation.desired_tags = {"traffic_sign"};
```

<div style="display: inline-block;"> 2.    Make the following changes and save the file. </div>
```cpp
    for (auto &bbox : annotation.second.bounding_boxes_2d)
            if(bbox.name == "face") {
                cv::rectangle(img, cv::Point(int(bbox.xmin), int(bbox.ymin)),
                cv::Point(int(bbox.xmax), int(bbox.ymax)),  
                cv::Scalar(0, 0, 255));
            }
```

<div style="white-space: pre-wrap;">
3. Build the client C++ file.

4. Press ‘Play’ on the monoDrive Scenario Editor. 

5. Debug & Run ‘camera_sensor.cpp’. The bounding boxes for the desired actors will be visible in the opencv window. </div>


## Modifying Bounding Boxes on a Streetlight

The goal of this section is to modify existing bounding boxes on a streetlight’s static mesh and display only the desired region of interest 

1.	Open the `Content Browser` on the monoDrive Scenario Editor. 

2.  Look for “prp_streetlight” under Content (see image below). Select prp_streetLight_Blueprint.
    <p class="img_container">
    <img class="wide_img" src="../img/stmesh_stlight_step2.png"/>
    </p>

3. Right click and select ‘Edit’. This opens the Blueprint editor window. 
    <p class="img_container">
    <img class="wide_img" src="../img/stmesh_stlight_step3.png"/>
    </p>

4.  Click on Viewport. 

    A.	Click on the streetlight. 
    
    B.	Double click on the Static Mesh component of the streetlight under 'Details' as shown in the image below. 
    <p class="img_container">
    <img class="wide_img" src="../img/stmesh_stlight_step4.png"/>
    </p>

5. Select Collision on the top of the screen and check the ‘Simple Collision’ box
    <p class="img_container">
    <img class="wide_img" src="../img/stmesh_stlight_step5.png"/>
    </p>

6. Once selected, all the bounding boxes for the current asset will be visible (as seen in the above image).

7.  Select bounding box 3 under Details&rarr;Collision&rarr;Boxes and expand it.

8.	Modify the Name parameter of Box 3. The image shown below illustrates the ‘Name’ updated as 'lamp'.    Box 3 is the lamp portion of the streetlight. 
    <p class="img_container">
    <img class="wide_img" src="../img/stmesh_stlight_step8.png"/>
    </p>

9.	Uncheck the ‘Contribute to Mass’ parameter so the box only serves annotation purposes. This disables the overall mass of the added shape. 


10.	Click on Save (top left corner). 

## Setting up the C++ client for a Streetlight

This section of the document explains how to modify code on the camera_sensor.cpp client example to specifically extract the above modified 2D bounding box with the label ‘lamp’ using annotations. 

<div style="white-space: inline-block;">
1. Navigate to monodrive-client&rarr;examples&rarr;cpp&rarr;dev&rarr;camera_sensor.cpp file. 
Make the following change to include the actor tag and save the file. Please note that skipping to next step will display all the existing bounding boxes for the “prp_streetLight” actor.
</div>
```cpp
    fc_config.annotation.desired_tags = {"street_light"};
```
<div style="display: inline-block;">
2. Make the following changes and save the file, 
to only view the one bounding box which was renamed. 
</div>
```cpp
    for (auto &bbox : annotation.second.bounding_boxes_2d)
               if(bbox.name == "lamp") {
                cv::rectangle(img, cv::Point(int(bbox.xmin), int(bbox.ymin)),
                              cv::Point(int(bbox.xmax), int(bbox.ymax)),  
                              cv::Scalar(0, 0, 255));
               }
```

<div style="white-space: pre-wrap;"> 

3. Build the client C++ file. 

4.	Press ‘Play’ on the monoDrive Scenario Editor. 

5.	Debug & Run ‘camera_sensor.cpp’. The bounding boxes for the desired actors will be visible in the opencv window.     
</div>

