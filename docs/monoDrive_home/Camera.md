# Camera

A monoDrive Camera Sensor can support six different image types:

- **RGB:** A three channel image of the scene
- **Grayscale:** A single channel grayscale image of the scene
- **Semantic Segmentation:** Each object in the scene is semantically labeled by color
- **Depth camera:** Provides a camera array where pixel values represent distance from the camera
- **Equidistant:** 
- **Poly1 Fisheye Camera:** Scaramuzza model Fisheye camera based.


Image output size, intrinsic camera parameters, and various other settings can 
be controlled through each camera's configuration.

## RGB Camera
Provides a RGBA camera stream with optional bounding boxes for dynamic objects in the scene.

<div class ='multi_img_container'>
<div class="wide_img">

``` json
   {
    "type": "Camera",
    "annotation": {
        "cull_partial_frame": false,
        "debug_draw": false,
        "desired_tags": [],
        "far_plane": 10000.0,
        "include_annotation": false,
        "include_obb": false,
        "include_tags": false
    },
    "channel_depth": 1,
    "channels": "bgra",
    "dynamic_range": 50.0,
    "focal_length": 9.0,
    "fov": 60.0,
    "fstop": 1.39999997615814,
    "listen_port": 8120,
    "location": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "max_shutter": 0.00139999995008111,
    "min_shutter": 0.000500000023748726,
    "rotation": {
        "pitch": 0.0,
        "roll": 0.0,
        "yaw": 0.0
    },
    "sensor_size": 9.06999969482422,
    "stream_dimensions": {
        "x": 512,
        "y": 512
    }
}
```
</div>

<p class="img_container">
  <img class='half_screen_img' src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/camerasensor.PNG"  height="400" />
</p>

</div>

- **stream_dimensions:** The size of the image in pixels
    - **x:** The width of the image in pixels
    - **y:** The height of the image in pixels
- **dynamic_range:** Controls the gain of the camera. A higher value will result in a grainer image.
- **fov:** Controls the horizontal angle of view of the camera. The vertical angle will be dynamically calculated based on `stream_dimensions`
- **focal_length:** If `fov` is not set explicitly, this will be used to emulate the focal length of the lens in millimeters. Used to calculate field-of-view.
- **fstop:** Controls the exposure time of the camera, higher values will yield brighter images with more motion blur.
- **min_shutter:** Lower bound of the camera's exposure time in seconds. Higher values will have more motion blur.
- **max_shutter:** Upper bound of the camera's exposure time in seconds. Higher values will have more motion blur.
- **sensor_size:** If `fov` is not set explicitly, this will be used to emulate the size of the camera's image sensor in millimeters. Used to calculate field-of-view.
- **channels:** Used to determine type of image output. For RGB cameras, this should always be `rgba`.
- **debug_draw:** If `true` and `annotation` is `true`, the bounding boxes will be displayed in the image for annotated objects.
- **annotation:** If `true`, the annotation information will be provided in the output data.
- **include_tags:** If `true`, the actor tag information will be included in annotations.
- **include_obb:** If `true`, the actor oriented bounding box information will be included in annotations.
- **cull_partial_frame:** If `true`, the actors that are only partially in frame will be removed from annotations.
- **far_plane:** The maximum distance in centimeters to annotate actors in the scene.
- **desired_tags:** If this array is not empty, the only actors with the tags specified here will be included in annotations.

## Grayscale Camera

Provides a grayscale camera stream with optional bounding boxes for dynamic objects in the scene.


<div class ='multi_img_container'>
<div class="wide_img">

``` json
   {
    "type": "Camera",
    "annotation": {
        "cull_partial_frame": false,
        "debug_draw": false,
        "desired_tags": [],
        "far_plane": 10000.0,
        "include_annotation": false,
        "include_obb": false,
        "include_tags": false
    },
    "channel_depth": 1,
    "channels": "gray",
    "dynamic_range": 50.0,
    "focal_length": 9.0,
    "fov": 60.0,
    "fstop": 1.39999997615814,
    "listen_port": 8120,
    "location": {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    },
    "max_shutter": 0.00139999995008111,
    "min_shutter": 0.000500000023748726,
    "rotation": {
        "pitch": 0.0,
        "roll": 0.0,
        "yaw": 0.0
    },
    "sensor_size": 9.06999969482422,
    "stream_dimensions": {
        "x": 512,
        "y": 512
    }
}
```

</div>

<p class="img_container">
  <img class='half_screen_img' src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/camera_grayscale.png"  height="400" />
</p>

</div>

All values are the same as the RGB camera except: 

- **channels:** Used to determine the type of image output. For grayscale cameras, this should always be `gray`.

<p>&nbsp;</p>

## Semantic Camera
Provides a grayscale camera stream where pixel values represent the semantic category of the rendered actor.

<div class ='multi_img_container'>
<div class="wide_img">

``` json
{
  "type": "SemanticCamera",
  "annotation": {
    "cull_partial_frame": false,
    "debug_draw": false,
    "desired_tags": [],
    "far_plane": 10000.0,
    "include_annotation": false,
    "include_obb": false,
    "include_tags": false
  },
  "channel_depth": 1,
  "channels": "gray",
  "focal_length": 9.0,
  "fov": 60.0,
  "listen_port": 8051,
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
  "sensor_size": 9.06999969482422,
  "stream_dimensions": {
      "x": 512,
      "y": 512
  },
  "debug_draw": false
}

```

</div>

<p class="img_container">
  <img class='half_screen_img' src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/semanticcamerasensor.PNG"  height="400" />
</p>

</div>

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

<div class ='multi_img_container'>
<div class="wide_img">

```json
{
    "type": "DepthCamera",
    "annotation": {
      "cull_partial_frame": false,
      "debug_draw": false,
      "desired_tags": [],
      "far_plane": 10000.0,
      "include_annotation": false,
      "include_obb": false,
      "include_tags": false
    },
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
  "debug_draw": false
 }
```
</div>

  <div class="img_container">
      <img class='half_screen_img' src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/camera_depth.png"/>
  </div>
  
</div>

These configuration values are the same as the RGB camera. Note that the output 
format of the image is an array the same size as `stream_dimensions` containing 
32-bit floating point numbers representing depth in centimeters.

## Raw Output

The total sensor output will be 12 bytes for the monoDrive sensor header plus 
the total number of bytes for the image defined as:

```bash
  Stream Dimension Width (x)  *  Stream Dimension Height (y)  *  4
```

for RGB images and 

```bash
  Stream Dimension Width (x)  *  Stream Dimension Height (y)
```

for grayscale images. See the "Depth Camera" description for information about 
the output format for this sensor.


## Equidistant Fisheye Camera

<div class ='multi_img_container'>
<div class="wide_img">

``` json
{
    "type": "EquidistantFisheyeCamera",
    "listen_port": 8100,
    "description": "",
	"location": {
	    "x": 0.0,
	    "y": 0.0,
	    "z": 225.0
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
	"annotation": {
		"cull_partial_frame": false,
		"debug_draw": false,
		"desired_tags": [],
		"far_plane": 10000.0,
		"include_annotation": false,
		"include_obb": false,
		"include_tags": false
	},
	"channel_depth": 1,
	"channels": "bgra",
	"dynamic_range": 50.0,
	"enable_streaming": true,
	"face_size": 1024,
	"fisheye_pixel_diameter": 512,
	"focal_length": 9.0,
	"fov": 180.0,
	"fstop": 1.399999976158142,
	"max_distance": 50000.0,
	"max_shutter": 0.00139999995008111,
	"min_shutter": 0.0005000000237487257,
	"sensor_size": 9.069999694824219,
	"viewport": {
		"enable_viewport": false,
		"fullscreen": false,
		 "monitor_name": "",
		"monitor_number": 0,
		 "window_offset": {
			"x": 0,
			"y": 0
		},
		"window_size": {
			"x": 0,
			"y": 0
		}
	},
	"vignette_bias": 0.5,
	"vignette_radius_start": 0.949999988079071,
	"wait_for_fresh_frame": true
}
```
</div>

<p class="img_container">
  <img class='half_screen_img' src="../img/equidistant.bmp"  height="400" />
</p>

</div>

The configuration for a Equidistant Fisheye Camera requires some additional settings.   

  - **vignette_bias:** Before the hard mechanical vignette at what point should the fade start normalized with respect to the diameter of the fisheye.     
  - **vignette_radius_start:** in the transition to black at the start of the mechanical vignette start with this value of black before the linear fade.    
  - **fisheye_pixel_diameter:** To obtain a bounded fisheye set the pixel diameter to the **smallest axis**, or to the measured real pixel diameter limit from the image. To obtain a diagonal bounded fisheye set the pixel diameter to the hypotenuse of the image. To obtain a fisheye that is greater than the image plane use the value from your model
  - **face_size:** Increasing this number improves image quality and vice versa with diminishing returns with respect to the image resolution. `face_size` should be **smaller** than the largest resolution.   

## Scaramuzza based Fisheye Camera 

<div class ='multi_img_container'>
<div class="wide_img">

``` json
{
    "type": "Poly1FisheyeCamera",
   	"listen_port": 8100,
   	"description": "",  
    "location": {
        "x": 0.0,
		"y": 0.0,
		"z": 225.0
	},
    "stream_dimensions": {
		"x": 512,
		"y": 512
	},
    "rotation": {
		"pitch": 0.0,
		"roll": 0.0,
		"yaw": 0.0
	},
    "viewport": {
		"enable_viewport": false,
		"fullscreen": false,
		"monitor_name": "",
		"monitor_number": 0,
        "window_offset": {
            "x": 0,
            "y": 0
        },
		"window_size": {
            "x": 0,
			"y": 0
		}
	},
	"a0": 349.1260070800781,
	"a2": -0.0010999999940395355,
	"a3": 1.1977999747614376e-06,
	"a4": -1.5119000496000012e-09,
	"channel_depth": 1,
	"channels": "bgra",
	"dynamic_range": 50.0,
	"enable_streaming": true,
	"face_size": 492,
	"focal_length": 9.0,
	"fov": 180.0,
	"fstop": 1.399999976158142,
	"max_distance": 50000.0,
	"max_shutter": 0.00139999995008111,
	"min_shutter": 0.0005000000237487257,
	"sensor_size": 9.069999694824219,
	"wait_for_fresh_frame": true,
    "annotation": {
        "cull_partial_frame": false,
        "debug_draw": false,
        "desired_tags": [],
        "far_plane": 10000.0,
        "include_annotation": false,
        "include_obb": false,
        "include_tags": false
	}
}
```
</div>

<p class="img_container">
  <img class='half_screen_img' src="../img/scaramuzza.bmp"  height="400" />
</p>

</div>

The configuration for a Poly1 Fisheye Camera requires some additional settings.    

- **"a0":**    
- **"a2":**    
- **"a3":**    
- **"a4":**   

### Annotation

``` json
[
    {
        "2d_bounding_boxes": [
            {
                "2d_bounding_box": [
                    158.12020874023438,
                    191.5050811767578,
                    261.953125,
                    272.0334167480469
                ],
                "name": "Body"
            }
        ],
        "name": "sedan_monoDrive_02_HD4_128",
        "oriented_bounding_box": [
            {
                "center": [
                    7072.78662109375,
                    -1290.6829833984375,
                    -172.56475830078125
                ],
                "extents": [
                    509.5,
                    184.18313598632812,
                    148.64846801757812
                ],
                "name": "Body",
                "orientation": [
                    -2.768632839433849e-06,
                    -0.00046827553887851536,
                    -0.9241809248924255,
                    0.381954550743103
                ],
                "scale": [
                    1.0,
                    1.0,
                    1.0
                ]
            }
        ],
        "tags": [
            "vehicle",
            "static",
            "car"
        ]
    },
    {
        "2d_bounding_boxes": [
            {
                "2d_bounding_box": [
                    189.89840698242188,
                    203.3860321044922,
                    260.5026550292969,
                    267.6765441894531
                ],
                "name": "Body"
            }
        ],
        "name": "subcompact_monoDrive_01_HD_32",
        "oriented_bounding_box": [
            {
                "center": [
                    9515.275390625,
                    -1273.10546875,
                    -172.4475555419922
                ],
                "extents": [
                    253.14500427246094,
                    144.71368408203125,
                    148.70858764648438
                ],
                "name": "Body",
                "orientation": [
                    8.400806109420955e-06,
                    -0.0004682083672378212,
                    -0.9330278635025024,
                    0.3598036468029022
                ],
                "scale": [
                    1.0,
                    1.0,
                    1.0
                ]
            }
        ],
        "tags": [
            "vehicle",
            "static",
            "car"
        ]
    },
]
```

Annotation data comes in an additional JSON message on the same port as the 
camera image data. Each element in the JSON array represents a single annotation 
for a dynamic actor.

- **2d_bounding_boxes:** Array of JSON bounding boxes for an actor
    - **2d_bounding_box:** Array containing the top-left x, top-left y, bottom-right x, bottom-right y coordinates in the image plane for the bounding box of this actor's section
    - **name:** The name of the actor's section this bounding box surrounds
- **name:** The name of the actor for these bounding boxes
- **oriented_bounding_box:** An array containing JSON for each 3D bounding box for the actor's sections
    - **center:** The x, y, and z center of the bounding box in centimeters with respect to the camera.
    - **extents:** The x, y, and z radius in centimeters from the center of the bounding box
    - **name:** The name of the actor's section this bounding box surrounds
    - **orientation:** The rotation of the bounding box about the center as a quaternion
    - **scale:** The scale coefficients for this bounding box
- **tags:** An array of tags assigned to this actor    


## Camera Coordinate Reference System
<p class="img_container">
<img class="lg_img" src="../img/camera_reference.png"/>
</p>



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
