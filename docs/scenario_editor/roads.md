# Import/Export Road Definitions

monoDrive import/export tool supports importing and exporting road definitions from/to OpenDrive format. You can use it to import a static road definition into an existing map or extract the OpenDrive file that is being used for the current map.

To save and load lane networks, use the monoDrive `RoadNetworkToolWidget` tool:

* Navigate to the root `Content` folder in the Scenario Editor. 

* Right-click the `RoadNetworkToolWidget` tool and choose "Run Editor Utility Widget":

<div class="img_container">
  <img class='wide_img' src="../imgs/lane_export_tool2.png"/>
</div>

* The `RoadNetworkToolWidget` tool dialog should now appear (likely docked in the top-right 
of your Editor window).

<div class="img_container">
  <img class='wide_img' src="../imgs/export_tool.png"/>
</div>

<p>&nbsp;</p>

## Import Road Definitions from File

OpenDrive can be loaded from file by: 

* Providing the path to an `.xodr` file into the "Filename" field and clicking "Load"

* The road definitions should appear in the `monoDriveRoadNetwork` directory in the "World Outliner"


## Exporting Road Definitions to File

After road definitions are created inside of a map, they can be exported to file:

1. Set the `Format` field to `opendrive`, we do not support geojson at the moment.

1. Select the appropriate coordinates system 

    - "world" coordinates use UTM coordinate frame

    - "gis" coordinates use the GPS coordinates anchored at the "GIS Anchor"

1. Set the orientation if there is a required orientation offset angle (yaw angle about z-axis)

1. Set the "Point Delta" field to the appropriate spacing between lane points in centimeters

1. Once all parameters are set, click the "Save" button and you data will be saved to specified file, or "Save As" to create a new file. 

<p>&nbsp;</p>