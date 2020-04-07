## Creating the Road Network

### Roads
The monoDrive Scenario Editor provides a tool for creating road networks that vehicles in the simulation
can drive on. 

To add a road to a map, use the __Actor Place Mode__ in the editor, and search for "monoDrive Road". Then simply drag and drop it into the scene.<br>
<img class='sm_img' src="https://github.com/monoDriveIO/documentation/raw/lane_tool/WikiPhotos/scenario_editor/roads/monoDrive_road_tool.PNG"/>

Roads are defined by a _reference line_ that represents the basic geometry of the road. Lanes can be added to a road in both the forward and reverse directions, and the lanes follow the path of the reference line. 

A forward lane begins at the location where the road is placed, and ends at the last spline point in the reference line. Similarly, reverse lanes begin at the last spline point and end at the location where the road was placed in the scene.

In the figure below, the blue line is the road's reference line. The blue dots are spline points along the road's reference line.

<img class='sm_img' src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/scenario_editor/roads/reference_line.PNG"/>

A road can have any number of forward and reverse lanes.

To add lanes, select the road in the editor, and click "Add Forward (right) Lane", or "Add Reverse (left) Lane" from the "Actions" section in the details pane. There are also buttons to remove lanes, as needed.

<img class='sm_img' src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/scenario_editor/roads/road_actions.PNG"/>

Roads can be connected to other roads as predecessors or successors. A road may only have a single predecessor and successor. However, a given lane can have any number of links to other roads' lanes.

The "Attach Road at Start/End" will create a new road at the specified location with the same number of lanes as the selected road, and will automatically link the lanes from each road to each other. The "Detach Road at Start/End" buttons will unlink the lanes from two connected roads, and remove the predecessor/successor link between them.

### Lanes

<img class='sm_img' src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/scenario_editor/roads/road_properties.PNG"/>
<img class='sm_img' src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/scenario_editor/roads/monoDrive_road_tool.PNG"/>
<img class='sm_img' src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/scenario_editor/roads/monoDrive_road_tool.PNG"/>
