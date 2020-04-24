# Camera

The monoDrive camera sensor supports four types of image output:

- Depth camera
- Grayscale
- RGB 
- Semantic Segmentation

The image output support different sizes i.e. 512x512 pixels.  
The location of the sensor can be modified in the "x", "y" and "z" axis with respect to the car.   
The sensor's orientation can be modified in the "yaw", "pitch" and "roll" axis. 
<p>&nbsp;</p>


## Depth Camera
Provides a grayscale camera stream with different intensity depending on the distance to the objects.

<div class="img_container">
    <img class='lg_img' src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/camera_depth.png"/>
</div>

```
[
  {
     "type": "DepthCamera",
      "listen_port": 8120,
      "location": {
       "x": -800.0,
       "y": 0.0,
       "z": 400.0
     },
     "rotation": {
       "pitch": -15.0,
       "yaw": 0.0,
       "roll": 0.0
     },
     "stream_dimensions": {
       "x": 512.0,
       "y": 512.0
     },
    "fov": 60.0
 }

]
```

<p>&nbsp;</p>


## Grayscale Camera
Provides a grayscale camera stream.

<p class="img_container">
  <img class='lg_img' src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/camera_grayscale.png"  height="400" />
</p>

```
[
  {
     "type": "Camera",
     "listen_port": 8120,
     "location": {
       "x": -800.0,
       "y": 0.0,
       "z": 400.0
     },
     "rotation": {
       "pitch": -15.0,
       "yaw": 0.0,
       "roll": 0.0
     },
     "stream_dimensions": {
       "x": 512.0,
       "y": 512.0
     },
    "max_distance": 50000.0,
    "dynamic_range":  50,
    "fov": 60.0,
    "focal_length": 9.0,
    "fstop": 1.4,
    "min_shutter":  0.000500,
    "max_shutter":  0.001400,
    "sensor_size":  9.07,
   "channels" : "gray"

 }

]
```

<p>&nbsp;</p>


## RGB Camera
Provides a RGBA camera stream with optional bounding boxes for dynamic objects in the scene.


<p class="img_container">
  <img class='lg_img' src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/camerasensor.PNG"  height="400" />
</p>

```
[
   {
    "type": "Camera",
    "listen_port": 8000,
    "location": {
      "x": 0.0,
      "y": 0.0,
      "z": 250.0
    },
    "rotation": {
      "pitch": 0.0,
      "yaw": 0.0,
      "roll": 0.0
    },
    "stream_dimensions": {
      "x": 512.0,
      "y": 512.0
    },
    "max_distance": 50000.0,
    "dynamic_range": 50,
    "fov": 60.0,
    "focal_length": 9.0,
    "fstop": 1.4,
    "min_shutter": 0.000500,
    "max_shutter": 0.001400,
    "sensor_size": 9.07,
    "channels": "rgba"
  }
]
```

<p>&nbsp;</p>


## Semantic Camera
Provides a grayscale camera stream.

<p class="img_container">
  <img class='lg_img' src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/semanticcamerasensor.PNG"  height="400" />
</p>

```
[
  {
     "type": "SemanticCamera",
     "listen_port": 8051,
     "location": {
       "x": -800.0,
       "y": 0.0,
       "z": 400.0
     },
     "rotation": {
       "pitch": -15.0,
       "yaw": 0.0,
       "roll": 0.0
     },
     "stream_dimensions": {
       "x": 512.0,
       "y": 512.0
     },
    "max_distance": 50000.0,
    "dynamic_range":  50,
    "fov": 60.0,
    "focal_length": 9.0,
    "fstop": 1.4,
    "min_shutter":  0.000500,
    "max_shutter":  0.001400,
    "sensor_size":  9.07,
   "channels" : "rgba"
 }

]
```

<p>&nbsp;</p>


## Configuration

### Annotation
All cameras support the annotation feature. The annotation feature will give you bounding boxes for all the dynamic objects in the scene. Add the following json tags to any camera configuration.
```
"annotation": {
    "desired_tags": [
    "traffic_sign", "vehicle"
    ],
    "far_plane": 10000.0,
    "include_tags": true
}
```
<p>&nbsp;</p>


### Configuration Tags

- **stream_dimensions**: The dimensions of which the stream will stream over TCP. These dimensions are set in a dictionary with keys x and y with float values.
- **dynamic_range**: The dynamic range of the camera in decibels.
- **fov**: (optional) If set this will override the angle of view calculated from the sensor size and lens focal length.
- **focal_length**: The focal length of the lens in millimeters.
- **fstop**: The f-stop value for the lens in stops.
- **min_shutter**: The minimum shutter speed, in seconds, for dynamic exposure.
- **max_shutter**: The maximum shutter speed, in seconds, for dynamic exposure.
- **sensor_size**: The size of the camera sensor in millimeters.
- **channels**: The output color of the camera, needs to be "rgba" or "grayscale".
- **desired_tags**: Tags of objects to be classified. Refer to the table below to obtain supported tags.

| Object  | Tag Name   |
| ------------ | ------------ |
|car  | car, vehicle |
|motorcycle | motorcycle, vehicle  |
|bus | bus, vehicle |
|person |  person, vru|
|bicycle | bicycle, vru |
|traffic signs | traffic_sign, tcd|

- **far_plane**: The maximum distance will be consider for annotation on the X axis of the camera. 
- **include_tags**: Flag to include the tags of the objects classified. 
- **include_obb**: Flag to include a 3D object-oriented bounding box in the output.
- **cull_partial_frame**: If set to true an object only partially visible in the camera frame will be toss out that annotation. 
<p>&nbsp;</p>


### Raw Output Data Format

- **time_stamp (int):** Timestamp representing milliseconds since Sunday.
- **game_time (float):** Current game time of simulator, this value will be more prominent.
- **Annotation (string):** Return information of the classified objects from the scene, the 2D bounding box, the 3D oriented bounding box, the tag name, etc.
- **image (List<List<float<float>>>):** A 3D list that represents a single image following the format of **stream_dimensions_x** x **stream_dimensions_y** x 4
<p>&nbsp;</p>


### Camera Configuration Examples

<p class="img_container">
  <img class='wide_img'src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/configuration/camera/cull_partial_frame.png" />
</p>

#### Field Of View Example

<p class="img_container">
  <img class='wide_img'src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/configuration/camera/FOV.png" />
</p>

#### Far Plane Example

<p class="img_container">
  <img class='wide_img'src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/configuration/camera/far_plane.png" />
</p>