# Python Examples

## monoDrive Python Examples

The monoDrive Python Client comes with several examples for connecting to and
using the the monoDrive Simulator and Scenario Editor. The examples in the
`examples` directory have many common elements that are discussed here.

### Connecting to the Simulator

To connect to the monoDrive Simulator, construct a `Simulator` object:

```python
simulator = Simulator.from_file(
    "examples/configurations/simulator_closed_loop.json",
    trajectory="examples/trajectories/Closed_Loop.json",
    sensors="examples/configurations/all_sensors.json',
    weather="examples/configurations/weather.json",
    ego="examples/configurations/vehicle.json"
)

simulator.start()
```

Here the simulator is being created from several of the provided configuration 
files for "Closed Loop" control of the ego vehicle. 

### Subscribing to Sensors

Once the client is connected to the `simulator` object, sensors can be 
configured and a callback function is connected to their output:

```python
simulator.subscribe_to_sensor('Camera_8000', camera_on_update)
```

Here, `Camera_8000` defines the unique identifier for the sensor which is a 
combination of the `type` field in `all_sensors.json` (i.e. `Camera`) and the
desired port number for the data (i.e. `8000`).

The callback function receives a packet of parsed camera data for use in 
perception algorithms. The callback can be as simple as:

```python
def camera_on_update(frame: CameraFrame):
    """
    callback to process parsed camera data
    """
    camera_frame = frame
    # Process the frame of camera data
```

### Sending Control Commands

In order to send vehicle controls to the `simulator` object for moving the EGO 
vehicle:

```python
forward = 1
right = 0
brake = 0
drive_mode = 1
simulator.send_control(forward, right, brake, drive_mode)
``` 

Here the `forward`, `right`, and `brake` values are floating point numbers 
between -1.0 and 1.0 to control the vehicle's movement. The `drive_mode` is set 
to `1` for closed loop control. Typically applications would want to control the 
vehicle in a continuous loop while simulating.

### Stopping the Simulator

When the application is complete, the `simulator` connection can be stopped:

```python
simulator.stop()
```

## Camera Annotation

The example code in the `examples/camera_annotation.py` demonstrates how to 
connect to an instance of the monoDrive Simulator and:

* Initiate "Replay" mode 
* Subscribe to a Camera sensor's output and process the annotations in the frame provided by the simulation
* Subscribe to a Semantic Camera sensor's output and display the camera data

Here, the simulation is being "stepped" in Replay mode instead of direct control
by providing a trajectory file and calling:

```python
simulator.step()
```

on each iteration of the simulation. 

The following code demonstrates how to utilize the annotation member of the 
`Camera` sensor:

```
if camera_frame:
    img = np.array(camera_frame.image[..., ::-1])
    for actor_annotation in camera_frame.annotation:
        for primitive_annotation in actor_annotation["2d_bounding_boxes"]:
            box = primitive_annotation["2d_bounding_box"]
            top_left = (int(box[0]), int(box[2]))
            bottom_right = (int(box[1]), int(box[3]))
            cv2.rectangle(img, top_left, bottom_right, (255, 0, 0), 2)
```


## Closed Loop Example

The example in `examples/closed_loop.py` demonstrates how to connect to an 
instance of the monoDrive Simulator and:

* Control the ego vehicle directly
* Subscribe to several different sensors and define their callbacks
* Process and display `Camera` sensor data
* Process and display `LiDAR` sensor data

Connecting to and controlling a vehicle in Closed Loop mode is demonstrated in 
the above descriptions. Processing and displaying `Camera` sensor data can be 
done like:

```python
if camera_frame:
    im = np.squeeze(camera_frame.image[..., ::-1])
    if data_camera is None:
        data_camera = ax_camera.imshow(im)
    else:
        data_camera.set_data(im)
```

Processing and displaying `LiDAR` sensor data can be done like:

```python
if lidar_frame:
    data = np.array([[pt.x, pt.y, pt.z, pt.intensity] for pt in lidar_frame.points])
    data = data[np.any(data != 0, axis=1)]
    if data_lidar is None:
        data_lidar = ax_lidar.scatter(
        data[:, 0], data[:, 1], data[:, 2],
        s=0.1
        )
    else:
        data_lidar._offsets3d = (data[:, 0], data[:, 1], data[:, 2])
```

## Replay Step Example

The example in `examples/replay_step.py` demonstrates how to connect to an 
instance of the monoDrive Simulator and:

* Initiate "Replay" mode and step the simulation
* Subscribe to a Camera sensor's output and display it
* Subscribe to a LiDAR sensor's output and display it

This example is very similar to the `camera_annotation.py` example.