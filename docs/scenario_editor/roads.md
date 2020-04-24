## Creating Road Networks

### Roads

The monoDrive Scenario Editor provides a "__monoDrive Road Tool__" 
for creating road networks that vehicles in the simulation can drive on. 

To add a road to a map, use the __Actor Place Mode__ in the editor, and search for "monoDrive Road". Then simply drag and drop it into the scene.

<div class="img_container">
  <img style="width: 100%" src="https://github.com/monoDriveIO/documentation/raw/lane_tool/WikiPhotos/scenario_editor/roads/place_road.gif"/>
</div>

Roads are defined by a _reference line_ that represents the basic geometry of the road. Lanes can be added to a road in both the forward and reverse directions, and the lanes follow the path of the reference line. 

A forward lane begins at the location where the road is placed, and ends at the last spline point in the reference line. Similarly, reverse lanes begin at the last spline point and end at the location where the road was placed in the scene.

In the figure below, the blue line is the road's reference line. The blue dots are spline points along the road's reference line. Forward lanes are on the right side of the reference line, and reverse lanes on the left side.

In addition, to get the _reference line_ facing the correct direction you road is going, press the 'e' key or tap the 'space bar' until the rotation tool is selected. Once this is done you may rotate the road to the direction you would like the cars to drive in.

<div class="img_container">
<img style="width: 100%" src="https://github.com/monoDriveIO/documentation/raw/lane_tool/WikiPhotos/scenario_editor/roads/reference_line.PNG"/>
</div>

A road can have any number of forward and reverse lanes.

To add lanes, select the road in the editor, and click "Add Forward (right) Lane", or "Add Reverse (left) Lane" from the "Actions" section in the details pane. There are also buttons to remove lanes, as needed.

<div class="img_container">
<img class='lg_img' src="https://github.com/monoDriveIO/documentation/raw/lane_tool/WikiPhotos/scenario_editor/roads/road_actions.PNG"/>
</div>

Roads can be connected to other roads as predecessors or successors. A road may only have a single predecessor and successor. However, a given lane can have any number of links to other roads' lanes.

The "Attach Road at Start/End" will create a new road at the specified location with the same number of lanes as the selected road, and will automatically link the lanes from each road to each other. 
The "Detach Road at Start/End" buttons will unlink the lanes from two connected roads, and remove the predecessor/successor link between them.

### Lanes

As mentioned above, lanes can either be forward lanes (they begin where the road was placed and end at the last spline point of the reference line), or reverse lanes. A lane is divided into _segments_, 
which is the area between two adjacent spline points. Lanes have several properties that can be edited from the details pane:

<div class="img_container">
<img class='lg_img' src="https://github.com/monoDriveIO/documentation/raw/lane_tool/WikiPhotos/scenario_editor/roads/lane_properties.PNG"/>
</div>

Lane segments also have several properties that can be edited as shown in the figure above.

The start and end offset for a lane specify the spline point at which a lane starts or ends. Depending on if you need to merge the start or end of the spline point, adjust the start or th end offset to a positive number, i.e. 2 as shown in the picture below. This is useful when creating lane merge scenarios.

<div class="img_container">
<img class='wide_img' src="https://github.com/monoDriveIO/documentation/raw/lane_tool/WikiPhotos/scenario_editor/roads/lane_offsets.PNG"/>
</div>

When a road is connected to another road as a successor or a predecessor, all available lanes on the predecessor are connected to those in the successor. This is done from left to right for forward lanes, 
and from right to left for reverse lanes. Lanes with a start/end offset are not included if the connection point is at the location of the offset.

Lanes can be connected to any number of other lanes by selecting the point of connection (either the start or the end), and selecting the corresponding link location. Lane connections can only be done at the 
start or end of a lane, so keep that in mind when building out the roads. If you're needing to connect a lane to the middle of some existing road, split that road into two roads so the connection can be at the start/end of a road.

<div class="img_container">
<img class='wide_img' src="https://github.com/monoDriveIO/documentation/raw/lane_tool/WikiPhotos/scenario_editor/roads/connecting_lanes.gif"/>
</div>

The red circle identifies the selected connection point for the road. The green circles identify existing connections. Possible connections are identified by the blue circles. Clicking on a connection creates the link between the two lanes.

To unlink lanes, right click on the red selected connection point and select "Detach Lane".

<div class="img_container">
<img class='lg_img' src="https://github.com/monoDriveIO/documentation/raw/lane_tool/WikiPhotos/scenario_editor/roads/disconnect_lane_menu.PNG"/>
</div>

<p>&nbsp;</p>

### Importing and Exporting

monoDrive uses GeoJSON to store and load road networks in the scenario creation tool and the simulator. Additionally, it supports importing and exporting from/to OpenDrive format. As of version 1.10, monoDrive 
supports version 1.5 of the OpenDrive specification.

To save and load lane networks, use the monoDrive `LaneExport` tool:

* Navigate to the root `Content` folder in the Scenario Editor. 

* Right-click the `LaneExport` tool and choose "Run Editor Utility Widget":

<div class="img_container">
  <img class='wide_img' src="../imgs/lane_export_run_option.png"/>
</div>

* The `LaneExport` tool dialog should now appear (likely docked in the top-right 
of your Editor window).

<div class="img_container">
  <img class='lg_img' src="../imgs/lane_export_tool.png"/>
</div>

<p>&nbsp;</p>

#### Loading Lanes from File

Both GeoJSON and OpenDrive can be loaded from file by: 

* Pasting the `.json` file or `.oxdr` file into the "Filename" field and click
"Load"

* The lanes should appear in the `SpawnedRoads` directory in the "World Outliner"


#### Exporting Lanes to File

After lanes are created inside of a map, they can be exported to file:

1. Set the file to save the data to `.oxdr` for OpenDrive or `.json` for GeoJSON

1. Select the appropriate file format from the `Format` field

1. Select the appropriate coordinates system 

    - "world" coordinates use UTM coordinate frame

    - "gis" coordinates use the GPS coordinates anchored at the "GIS Anchor"

1. Set the orientation if there is a required orientation offset angle (yaw angle about z-axis)

1. Set the "Point Delta" field to the appropriate spacing between lane points in centimeters

1. Once all parameters are set, click the "Save" button and you data will be saved to specified file

<p>&nbsp;</p>

<!-- #### Demos

To find more examples of the information above, follow the links below to our "__monoDrive Road Tool__" Demos:

(Place Demos here once done.) -->