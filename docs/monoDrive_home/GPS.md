# GPS

The GPS coordinates are based on the GPS anchor set for the current map. The location and orientation are relative to the ego vehicle's origin.

## Configuration 

``` json
{
  "type": "GPS",
  "listen_port": 8100,
  "location": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0
  },
  "rotation": {
      "pitch": 0.0,
      "roll": 0.0,
      "yaw": 0.0
  },
  "anchor_overrides": {
      "gis_anchor": {
          "x": 0.0,
          "y": 0.0,
          "z": 0.0
      },
      "rotation": -90.0,
      "use_override": false,
      "world_anchor": {
          "x": -97.7431,
          "y": 30.2672,
          "z": 0.0
      }
  },
  "description": ""
}

```

- **anchor_overrides**:   
    - **use_override**: if set to true use these override values, if false the **gis_anchor** actor placed in the map will be used, if there is no **gis_anchor** actor and this is set to false then the Center of Austin Texas will be used at your maps coordinate center.
    - **gis_anchor**: (longitude, latitude, height) positioned at the world anchor
    - **rotation**: the north direction for heading rotated counter clockwise with respect to the X vector of the map
    - **world_anchor**: the position in the simulation map where the gis_anchor originates    
  
  
## Adding a GPSAnchor to a map
1. On the search box of the `Place Actors` window, look for `GPSAnchor`. 
2. Drag and drop to the map at the desired location. 
3. On the `Details` windows, modify the `x` for longitude and `y` for latitude, as desired.

<p class="img_container">
  <img class="wide_img" src="../img/GPSAnchor.png" />
</p>


## Output

The total sensor output is 92 bytes, where the first 16 bytes correspond to the 
monoDrive sensor header and the remaining 76 conform the GPS information.
Below is a table of each element in the message:

| Type  | Name   | Units   | Description   |
| ------------ | ------------ |------------ | ------------ |
|Float | Latitude | Degrees | Latitude on the map with respect to the GIS anchor |
|Float | Longitude | Degrees | Longitude on the map with respect to the GIS anchor |
|Float | Elevation | Meters | Elevation on the map with respect to the GIS anchor |
|Float | World Location x | Cemtimeter | Coordinate in the y direction with respect to the world |
|Float | World Location y| Cemtimeter| Coordinate in the x direction with respect to the  world |
|Float | Forward x   | | X componet of the GPS anchor unit vector |
|Float | Forward y   | | Y componet of the GPS anchor unit vector |
|Float | Forward z   | | Z componet of the GPS anchor unit vector|
|Float | Ego yaw  | Degrees | Compass heading of the ego vehicle |
|Float | Ego speed | Meters per second | Speed of the ego vehicle |
|U16 | horizontal accuracy  | Meters | Horizontal accuracy of the GPS|
|U16 | vertical accuracy  | Meters |  Vertical accuracy of the GPS|
|U8| Number of satellites| | Number of satellites used for signal |
|U8 | Mode| | Fixed mode status |
|U16| CRC  | | |

<p>&nbsp;</p>