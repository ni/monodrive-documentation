# RSBR  

The Radar Shooting Bouncing Ray (RSBR) sensor provides ground truth hit point 
access to targets that are within the field-of-view of a simulated radar. The 
output of the sensor provides target intersection information at the resolution 
and field-of-view specified by the configuration. 

## Configuration

``` json
{
    "type": "RSBR",
    "listen_port": 8123,
    "location": {
        "x": 250.0,
        "y": 0.0,
        "z": 50.0
    },
    "rotation": {
        "pitch": 0.0,
        "roll": 0.0,
        "yaw": 0.0
    },
    "azi_fov": 45.0,
    "ele_fov": 45.0,
    "consider_ego": false,
    "consider_road": false,
    "road_sample_density": 1000.0,
    "max_bounce_count": 2,
    "sample_density": 1000.0,
    "scan_distance": 3000.0,
    "draw_los_lines": false,
    "draw_points": false,
    "draw_reflection_bounce": 1,
    "draw_reflection_lines": false,
    "draw_road_points": false,
}
```

- **azi_fov:** The horizontal field-of-view along the azimuth of the sensor in degrees.
- **ele_fov:** The vertical field-of-view along the elevation of the sensor in degrees.
- **consider_ego:** If set to true then the ego will reflect points the same as other actors.
- **consider_road:** If set to true then hit points from the road will also be sampled.
- **road_sample_density:** The density in cm^2 / point to sample the road. 
- **max_bounce_count:** The total number of bounces to sample per ray.
- **sample_density:** The density in cm^2 / point to sample actors.
- **scan_distance:** The maximum distance the sensor will detect objects in centimeters.
- **draw_los_lines:** If true then the line-of-sight lines (direct path to sensor) will be drawn.
- **draw_points:** If true then the sample points for each actor will be drawn around the bounding box.
- **draw_reflection_bounce:** When `draw_reflection_lines` is true, this will be the bounce that is drawn for the lines (e.g. 1, 2, 3, etc.).
- **draw_reflection_lines:** If true then the reflection lines will be drawn for the selected bounce.
- **draw_road_points:** If true then the points sampled from the road will be drawn.

## Raw Output

The RSBR dataframe contains a vector of `RSBRHitPoint`s. The directions and normals are within the RSBR sensor's coordinate frame. Here is the information for a single hit point.

- **distance:** The magnitude of the vector from the sensor to the target in meters.
- **point:** The 3-D point for the final sample position.
- **direction:** A unit vector representing the 3-D direction of the ray cast.
- **normal:** A unit vector representing the 3-D direction of the normal to the surface sampled.
- **relative_velocity:** The relative velocity of the target to the sensor.
- **roughness:** The roughness value for the sampled surface.
- **dielectric_constant:** The dielectric constant for the sampled surface.
- **radial_velocity:** The velocity of the sampled target across the azimuth direction.
- **reflection:** The total reflection value off the sampled surface.
- **point_id:** The unique ID of this hit point.
- **parent_point_id:** The unique ID of the origin point that was the origin for this sample. A value of `0` indicates the point has no parent's and has an origin at the sensor. 
- **bounce_count:** The total number of bounces from the sensor before the sampled surface was impacted.
- **is_direct:** If true then there exists a direct line of sight from this point to the sensor origin.
- **in_fov:** If true then this point falls within the original field-of-view of the sensor.
- **object_id:** The unique ID of the actor that was sampled.
- **object_center:** The center 3-D point of the actor that is sampled.
- **object_orientation:** The orientation vector of the sampled actor.
- **object_extents:** The extents from the `object_center` of the sampled actor's bounding box.
- **isRoad:** If true then the sampled actor was designated as a road surface.

**NOTE:** The lineage of a point is a graph and not a single ray bouncing. To trace a point's lineage just continually follow the `parent_point_id` up the chain until you reach a value of `0` indicating this is the origin position of the tree. 

<p>&nbsp;</p>

<div class="img_container">
  <video width=650px height=238px class="border" muted autoplay loop>
    <source src="https://cdn.monodrive.io/readthedocs/rsbr.mp4" type="video/mp4">
  </video>
</div> 

<p>&nbsp;</p>