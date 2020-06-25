# Occupancy Grid

## Occupancy Grid

The monoDrive occupancy grid sensor provides a top-down orthographic view
of the semantic information in a simulation. The sensor follows the ego vehicle either locked to horizontal and vertical axis of the world, or rotating with 
the vehicle. Each pixel in the image is square and is exactly the size set
by the input JSON configuration. 

<div class="img_container">
    <img class='lg_img' src="../img/occupancy_grid.png"/>
</div>


## Configuration

``` json
{
	"type": "OccupancyGrid",
	"listen_port": 0,
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
	"stream_dimensions": {
		"x": 512,
        "y": 512
	},
	"follow_pitch": false,
	"follow_roll": false,
	"follow_yaw": false,
	"meters_per_pixel": 0.1
}
```

- **stream_dimensions:** The size of the grid that will be created
    - **x:** The number of pixels in the horizontal dimension
    - **y:** The number of pixels in the horizontal dimension
- **follow_pitch:** If true, the sensor will pitch by the pitch angle of the ego vehicle.
- **follow_roll:** If true, the sensor will roll by the roll angle of the ego vehicle.
- **follow_yaw:** If true, the sensor will yaw by the yaw angle of the ego vehicle.
- **meters_per_pixel:** The number of meters each pixel in the grid will represent.

## Raw Output

The total sensor output will be 12 bytes for the monoDrive sensor header plus the total number of bytes for the image defined as:

```bash
  Stream Dimension Width (x)  *  Stream Dimension Height (y)
```