# Waypoint Sensor

The waypoint sensor returns a JSON string with information on the waypoints behind and in front of all the vehicles in the scenario, given the specified distance and frequency. Information about the left and right lanes is also included.    

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
**distance** : Maximum distance for the waypoint to return waypoints.   
**frequency** : Defines how apart the waypoints are from each other.    
**draw_debug** : Set to true if you want to visualize the waypoints during simulation.   
**debug_tags**: If `draw_debug` is true, the waypoints will be drawn only for the vehicles that contain the same tags specified here.   
