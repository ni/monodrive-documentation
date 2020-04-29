# Camera

A monoDrive Camera Sensor can support four different image types:

- **RGB:** A three channel image of the scene
- **Grayscale:** A single channel grayscale image of the scene.
- **Semantic Segmentation:** Each object in the scene is semantically labeled by color
- **Depth camera:** Provides a camera array where pixel values represent distance from the camera

Image output size, instrinsic camera parameters, and various other settings can 
be controlled through each camera's configuration.

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
    "dynamic_range": 50,
    "fov": 60.0,
    "focal_length": 9.0,
    "fstop": 1.4,
    "min_shutter": 0.000500,
    "max_shutter": 0.001400,
    "sensor_size": 9.07,
    "channels": "rgba",
    "debug_draw": false,
    "annotation": false,
    "include_tags": false,
    "include_obb": false,
    "cull_partial_frame": false,
    "far_plane": 5000.0,
    "desired_tags": [
      "dynamic"
    ]
  }
]
```

- **stream_dimensions:** The size of the image in pixels
    - **x:"** The width of the image in pixels
    - **y:"** The height of the image in pixels
- **dynamic_range:** Controls the gain of the camera. A higher value will result in a grainer image.
- **fov:** Controls the horizontal angle of view of the camera. The vertical angle will be dynamically calculated based on `stream_dimensions`
- **focal_length:** If `fov` is not set explicitly, this will be used to emulate the focal lenght of the lens, in millimeters. Used to calculate field-of-view.
- **fstop:** Controls the exposure time of the camera, higher values will yield brighter images with greater motion blur.
- **min_shutter:** Lower bound of the camera exposure time in seconds. Higher values give more motion blur.
- **max_shutter:** Lower bound of the camera exposure time in seconds. Higher values give more motion blur.
- **sensor_size:** If `fov` is not set explicitly, this will be used to emulate the size of the camera's image sensor, in millimeters. Used to calculate field-of-view.
- **channels:** Used to determine type of image output. For RGB cameras, this should always be `rgba`.
- **debug_draw:** If `true` and `annotation` is `true`, then bounding boxes will be displayed in the image for annotated objects.
- **annotation:** If `true`, then annotation information will be provided in the output data.
- **include_tags:** If `true`, then actor tag information will be included in annotations.
- **include_obb:** If `true`, then actor oriented bounding box information will be included in annotations.
- **cull_partial_frame:** If `true`, then actors that are only partially in frame will be removed from annotations.
- **far_plane:** The maximum distance, in centimeters, to annotate actors in the scene.
- **desired_tags:** If this array is not empty, then only actors with the tags specified here will be included in annotations.

## Grayscale Camera
Provides a grayscale camera stream with optional bounding boxes for dynamic objects in the scene.

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
    "dynamic_range":  50,
    "fov": 60.0,
    "focal_length": 9.0,
    "fstop": 1.4,
    "min_shutter":  0.000500,
    "max_shutter":  0.001400,
    "sensor_size":  9.07,
    "channels" : "gray",
    "debug_draw": false,
    "annotation": false,
    "include_tags": false,
    "include_obb": false,
    "cull_partial_frame": false,
    "far_plane": 5000.0,
    "desired_tags": [
      "dynamic"
    ]
 }
]
```
All values are the same as the RGB camera except: 

- **channels:** Used to determine type of image output. For RGB cameras, this should always be `grayscale`.

<p>&nbsp;</p>

## Semantic Camera
Provides a grayscale camera stream where pixel values represent the semantic category of the rendered actor.

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
    "fov": 60.0,
    "focal_length": 9.0,
    "sensor_size":  9.07,
    "channels" : "gray",
    "debug_draw": false,
    "annotation": false,
    "include_tags": false,
    "include_obb": false,
    "cull_partial_frame": false,
    "far_plane": 5000.0,
    "desired_tags": [
      "dynamic"
    ]
 }
]
```

The configuration values are the same as RGB. The following table shows the 
semantic definition for each pixel value:

| Asset |	Grayscale Pixel Value | 
| ----- | --------------------- |
| ego vehicle	| 2 |
| car |	3 | 
| motorcycle |	4 |
| bus |	6 |
| truck |	8 |
| fence/guardrail |	5 |
| traffic light |	10 |
| person |	11 |
| bicycle |	12 |
| building (shipping containers) |	15 |
| traffic signs |	20 |
| lane markers |	70 |
| terrain |	80 |
| foliage |	85 |
| gravel |	100 |
| power lines |	110 |
| pylons |	115 |
| sky |	141 |
| street light/pole |	153 |
| road |	175 |
| sidewalk |	190 |
| road art |	193 |


## Depth Camera
Provides an unsigned 32-bit floating point array where the pixel values 
represent the distance from the camera in centimeters.

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
    "fov": 60.0,
    "debug_draw": false,
    "annotation": false,
    "include_tags": false,
    "include_obb": false,
    "cull_partial_frame": false,
    "far_plane": 5000.0,
    "desired_tags": [
      "dynamic"
    ]
 }
]
```

These configuration values are the as the RGB camera. Note that the output format
of the image is an array the size of the `stream_dimensions` containing 32-bit
floating point numbers representing depth in centimeters.

## Raw Output

- **time_stamp:** 32-bit integer timestamp representing milliseconds since Sunday.
- **game_time:** 32-bit floating point number representing the current game time of simulator.
- **Annotation:** JSON string with information of the classified objects from the scene, the 2D bounding box, the 3D oriented bounding box, the tag name, etc.
- **image:** An array that is the shape of `stream_dimensions` with a depth 1 for grayscale images and 4 for RGB images. See the "Depth Camera" description for information about the output format for this sensor.

### Annotation
All cameras support the annotation feature. The annotation feature will give 
you bounding boxes for all the dynamic objects in the scene. Add the following 
JSON tags to any camera configuration.
```
"annotation": {
    "desired_tags": [
    "traffic_sign", "vehicle"
    ],
    "far_plane": 10000.0,
    "include_tags": true
}
```

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