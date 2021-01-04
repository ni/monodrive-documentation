## Importing and Exporting

monoDrive uses GeoJSON to store and load road networks in the scenario creation tool and the simulator. Additionally, it supports importing and exporting from/to OpenDrive format. As of version 1.10, monoDrive 
supports version 1.5 of the OpenDrive specification.

To save and load lane networks, use the monoDrive `LaneExport` tool:

* Navigate to the root `Content` folder in the Scenario Editor. 

* Right-click the `LaneExport` tool and choose "Run Editor Utility Widget":

<div class="img_container">
  <img class='wide_img' src="../imgs/lane_export_tool2.png"/>
</div>

* The `LaneExport` tool dialog should now appear (likely docked in the top-right 
of your Editor window).

<div class="img_container">
  <img class='wide_img' src="../imgs/lane_export_tool1.png"/>
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