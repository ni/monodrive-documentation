# Waypoint

The waypoint sensor returns a JSON string with information on the waypoints behind and in front of all vehicles in the scenario. Information for the vehicle's current lane, left lane, and right lane are included.

<p class="img_container">
<img class="lg_img" src="../img/waypoint.png"/>
</p>


## Configuration

``` json
{
    "type": "Waypoint",
    "listen_port": 8556,
    "debug_tags": ["ego"],
    "distance": 1000.0,
    "frequency": 100.0,
    "draw_debug": true
}

```

 - **distance** : Distance in centimters to generate waypoints from the vehicle's position.
 - **frequency** : Distance in centimeters that specifies the spacing between waypoints.
 - **draw_debug** : Set to "true" to visualize the waypoints during simulation.
 - **debug_tags**: If `draw_debug` is true, the waypoints will be drawn only for the vehicles that contain the same tags specified here.   
