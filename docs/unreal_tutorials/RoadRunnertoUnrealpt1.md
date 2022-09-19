# RoadRunner Example Pipeline

This is a guide for creating roads in RoadRunner and how to export them in preparation for Unreal Engine. Please note that Roadrunner uses meters as its base measurement scale, and that it is suggested to import a vehicle for help with scaling in the scene.

## Road Creation Basics

1. Open RoadRunner and create a new scene/map
    <div class="img_container">
      <img class="extra_lg_img" src="../imgs/RoadRunnerPic_01.png"/>
    </div>
1. Select the "Road Plan Tool" icon located in the top left
  <div class="img_container">
      <img class="md_img" src="../imgs/RoadRunnerPic_02.png"/>
  </div>
1. Navigate to the "Library Browser" in the bottom right
    - Navigate to the folder labeled "Road Styles"
    - Select road type
  <div class="img_container">
      <img class="lg_img" src="../imgs/RoadRunnerPic_03.png"/>
  </div>
1. Now right click on the grid, this will create a red dot
    - When right clicking again in another location within the screen space, this will create the basic road.

## Create a Landscape

1. With the road drawn, create a landscape around the road.

1. Select the "Surface Tool"
  <div class="img_container">
      <img class="sm_img" src="../imgs/RoadRunnerPic_04.png"/>
  </div>
1. In the screen gridded space, right click on the outside of where the road is located.
  - Make adjustments as needed by clicking on the dots
  <div class="img_container">
      <img class="lg_img" src="../imgs/RoadRunnerPic_05.png"/>
  </div>
1. Continue adding dots to create a circle until you reach the original dot that was placed, this will create a landscape around the road.
  <div class="img_container">
      <img class="lg_img" src="../imgs/RoadRunnerPic_06.png"/>
  </div>

## Export to Unreal Engine

1. Start the export process by navigating to "File" → "Export" → "Unreal"
  <div class="img_container">
      <img class="wide_img" src="../imgs/RoadRunnerPic_07.png"/>
  </div>
1. Click on "Unreal", another window will pop up
    - Navigate a place to save the created roads, it is recommended to create a new folder
    - The default setting for exporting .fbx files should be adequate.
    - Once all adjustments are made, click "Export"
  <div class="img_container">
      <img class="lg_img" src="../imgs/RoadRunnerPic_08.png"/>
  </div>
1. For the vehicles to be drivable in Unreal Engine, export the OpenDrive file. Go to "File" → "Export" → "OpenDrive"
1. Select the same location of the "Unreal" export
1. Click "Export"
  <div class="img_container">
      <img class="wide_img" src="../imgs/RoadRunnerPic_09.png"/>
  </div>
  
  <div class="img_container">
      <img class="md_img" src="../imgs/RoadRunnerPic_10.png"/>
  </div>