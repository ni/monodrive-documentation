## Creating Road Networks

### Roads
The monoDrive Scenario Editor provides a "__monoDrive Road Tool__" 
for creating road networks that vehicles in the simulation can drive on. 

To add a road to a map, use the __Actor Place Mode__ in the editor, and search for "monoDrive Road". Then simply drag and drop it into the scene.

<img class='wide_img' src="https://github.com/monoDriveIO/documentation/raw/lane_tool/WikiPhotos/scenario_editor/roads/monoDrive_road_tool.PNG"/>

Roads are defined by a _reference line_ that represents the basic geometry of the road. Lanes can be added to a road in both the forward and reverse directions, and the lanes follow the path of the reference line. 

A forward lane begins at the location where the road is placed, and ends at the last spline point in the reference line. Similarly, reverse lanes begin at the last spline point and end at the location where the road was placed in the scene.

In the figure below, the blue line is the road's reference line. The blue dots are spline points along the road's reference line.

<img class='wide_img' src="https://github.com/monoDriveIO/documentation/raw/lane_tool/WikiPhotos/scenario_editor/roads/reference_line.PNG"/>

A road can have any number of forward and reverse lanes.

To add lanes, select the road in the editor, and click "Add Forward (right) Lane", or "Add Reverse (left) Lane" from the "Actions" section in the details pane. There are also buttons to remove lanes, as needed.

<img class='lg_img' src="https://github.com/monoDriveIO/documentation/raw/lane_tool/WikiPhotos/scenario_editor/roads/road_actions.PNG"/>

Roads can be connected to other roads as predecessors or successors. A road may only have a single predecessor and successor. However, a given lane can have any number of links to other roads' lanes.

The "Attach Road at Start/End" will create a new road at the specified location with the same number of lanes as the selected road, and will automatically link the lanes from each road to each other. The "Detach Road at Start/End" buttons will unlink the lanes from two connected roads, and remove the predecessor/successor link between them.

### Lanes

As mentioned above, lanes can either be forward lanes (they begin where the road was placed and end at the last spline point of the reference line), or reverse lanes. A lane is divided into _segments_, which is the area between two adjacent spline points. Lanes have several properties that can be edited from the details pane:

<img class='lg_img' src="https://github.com/monoDriveIO/documentation/raw/lane_tool/WikiPhotos/scenario_editor/roads/lane_properties.PNG"/>

Lane segments also have several properties that can be edited as shown in the figure above.

The start and end offset for a lane specify the spline point at which a lane starts or ends. This is useful when creating lane merge scenarios as in the picture below.

<img class='wide_img' src="https://github.com/monoDriveIO/documentation/raw/lane_tool/WikiPhotos/scenario_editor/roads/lane_offsets.PNG"/>

When a road is connected to another road as a successor or a predecessor, all available lanes on the predecessor are connected to those in the successor. This is done from left to right for forward lanes, and from right to left for reverse lanes. Lanes with a start/end offset are not included if the connection point is at the location of the offset.

Lanes can be connected to any number of other lanes by selecting the point of connection (either the start or the end), and selecting the corresponding link location. Lane connections can only be done at the start or end of a lane, so keep that in mind when building out the roads. If you're needing to connect a lane to the middle of some existing road, split that road into two roads so the connection can be at the start/end of a road.

<img class='wide_img' src="https://github.com/monoDriveIO/documentation/raw/lane_tool/WikiPhotos/scenario_editor/roads/connecting_lanes.gif"/>

The red circle identifies the selected connection point for the road. The green circles identify existing connections. Possible connections are identified by the blue circles. Clicking on a connection creates the link between the two lanes.

To unlink lanes, right click on the red selected connection point and select "Detach Lane"

<img class='lg_img' src="https://github.com/monoDriveIO/documentation/raw/lane_tool/WikiPhotos/scenario_editor/roads/disconnect_lane_menu.PNG"/>

