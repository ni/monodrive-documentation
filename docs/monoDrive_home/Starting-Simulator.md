## Client Control Flow with Simulator

<img src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/ControlFlow.PNG" width="600" height="400" />
</p>

1. Control:
    - Create an instance of the `Simulator` class with the desired configuration and send the configuration to the simulator:
        ```
        simulator = Simulator(SimulatorConfiguration('simulator.json'))
        simulator.send_configuration()
        ```
    - Get an instance of a vehicle of the desired type (`TeleportVehicle` is driven by the user, `SimpleVehicle` uses the `Waypoint` and `GPS` sensors to drive on it's own) and cofiguration:
        ```
        from monodrive.vehicles import TeleportVehicle
        ...
        vehicle_config = VehicleConfiguration('demo.json')
        ego_vehicle = simulator.get_ego_vehicle(vehicle_config, TeleportVehicle)
        ```
          or
        ```
        from monodrive.vehicles import SimpleVehicle
        ...
        vehicle_config = VehicleConfiguration('demo.json')
        ego_vehicle = simulator.get_ego_vehicle(vehicle_config, SimpleVehicle)
        ```
          _Note: the simulator currently supports only one ego vehicle instance per simulation session_

    - Send the vehicle configuration and start the vehicle process:
        ```
        simulator.send_vehicle_configuration(vehicle_config)
        ego_vehicle.start()
        ```
2. Simulator: Each sensor will sample it's data and send it to the client.

3. Sensor: Each sensor in the client will digest this data from the simulator and put the parsed data into both `q_display` and `q_data`. `q_display` is used for displaying the sensor's data in a graph, `q_data` is used in the Vehicle class to assist the ego vehicle in driving.
    ```
            for sensor in ego_vehicle.sensors:
                data = sensor.get_message()
                # do something interesting with the data
    ```

4. Vehicle: The `SimpleVehicle` class implements `drive(sensors)` to use the sensors' data to calculate control values for the ego vehicle. These control values are sent to the simulator to control the vehicle.

The JSON format the simulator expects to control the ego vehicle.

```
{
    "forward_amount": float,
    "right_amount": float
}
```
* __forward_amount__: Value from -1.0 to 1.0, negative being reverse
* __right_amount__: Value from -1.0 to 1.0, negative being left

