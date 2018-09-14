## Client Control Flow with Simulator

<img src="https://github.com/monoDriveIO/PythonClient/blob/master/WikiPhotos/ControlFlow.PNG" width="600" height="400" />
</p>

1. Control:
- At Start: Start the simulator off by sending an empty control command to get an initial read off the sensor values. This is done in the `SensorManager.start()`, which is called in `BaseVehicle.start()`.
- After Start: Use the `VehicleController` class to send control data to the simulator. Follow the JSON format documented below.

2. Simulator: Each sensor will sample it's data and send it to the client.

3. Sensor: Each sensor in the client will digest this data from the simulator and put the parsed data into both `q_display` and `q_vehicle`. `q_display` is used for displaying the sensor's data in a graph, `q_vehicle` is used in the Vehicle class to assist the Ego vehicle in driving.

4. Vehicle: The vehicle class implements `plan_control_data(sensors)` to use the sensors' data to calculate control values for the Ego vehicle. These control values are sent to the simulator using the `VehicleController` class.

The JSON format the simulator expects to control the ego vehicle.

```
{
    "forward_amount": float,
    "right_amount": float,
    "update_vehicle_position": int,
    "current_lane": int
}
```
* __forward_amount__: Value from -1.0 to 1.0, negative being reverse
* __right_amount__: Value from -1.0 to 1.0, negative being left
* __update_vehicle_position__: Value to update the Waypoint sensor based on how many points the vehicle has surpassed (read more below)
* __current_lane__: Updates the simulator on the current lane of the ego vehicle, used for sensing vehicles in the Bounding Box sensor

#### Update Vehicle Position Explained
This is specifically for the Waypoint sensor. The Waypoint sensor starts reading data where the ego vehicle
spawns, in order to move where the Waypoint sensor starts getting waypoints from, `update_vehicle_position` must be
used. The `update_vehicle_position` value represents how many waypoints the sensor has passed. So if
the ego has driven past 10 of the first 100 waypoints, we can have a value of 10 for `update_vehicle_position` and the
Waypoint sensor will start reading waypoints from 10 points ahead of the original spawn position. In our example
we wait for the ego to get past half of the points, and then we tell the waypoint to move forward / update 
that exact amount. If 100 points come down in the waypoint sensor (parameter in the Waypoint sensor
`total_points`) we get points 1-100. We wait for the ego to get through 50 of those and then we send
`update_vehicle_position` = 50, which means the new waypoint data will start from that 51st spot and have points all
the way from 51 - 150. Once we do this again, we send another value of 50 (getting through 50 more points), we 
set `update_vehicle_position` to 50 again and the next set of waypoints we get will be 101-200.