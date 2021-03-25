# Camera

A monoDrive Camera Sensor can support six different image types:

- **RGB:** A three channel image of the scene
- **Grayscale:** A single channel grayscale image of the scene
- **Semantic Segmentation:** Each object in the scene is semantically labeled by color
- **Depth camera:** Provides a camera array where pixel values represent distance from the camera
- **Fisheye:** Equidistant or Poly1 Fisheye Camera/Scaramuzza model Fisheye camera based.
- **360 Camera:** Provides a 360 view of the scene.

Image output size, intrinsic camera parameters, and various other settings can 
be controlled through each camera's configuration.

## RGB Camera
Provides a RGBA camera stream with optional bounding boxes for dynamic objects in the scene.

<div class ='multi_img_container'>
<div class="wide_img">

``` json
{
    "type":"Camera",
    "listen_port":8010,
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
    "min_shutter": 0.0005,
    "max_shutter": 0.0014,
    "sensor_size": 9.07,
    "color_filter_array": {
        "use_cfa": false,
        "cfa":"rccc"
    },
    "channels": "bgra",
    "viewport": {
        "enable_viewport": false,
        "fullscreen": false,
            "monitor_name": "",
        "monitor_number": 0,
        "window_offset": {
            "x": 32,
            "y": 32
        },
        "window_size": {
            "x": 0,
            "y": 0
        }
    },
    "ray_tracing_enable": false,
    "annotation": {
      "include_annotation": false,
      "desired_tags": [
        "traffic_sign", "vehicle"
      ],
      "far_plane": 10000.0,
      "include_tags": true,
      "include_obb": false,
      "cull_partial_frame":false,
      "debug_draw":false
    },
    "max_distance":50000.0,
    "channel_depth":1
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
- **focal_length:** If `fov` is set to `0`, this will be used to emulate the focal length of the lens in millimeters. Used to calculate field-of-view.
- **fstop:** Controls the exposure time of the camera, higher values will yield brighter images with more motion blur.
- **min_shutter:** Lower bound of the camera's exposure time in seconds. Higher values will have more motion blur.
- **max_shutter:** Upper bound of the camera's exposure time in seconds. Higher values will have more motion blur.
- **sensor_size:** If `fov` is set to `0`, this will be used to emulate the size of the camera's image sensor in millimeters. Used to calculate field-of-view.
- **channels:** Used to determine type of image output. For RGB cameras, this should always be `rgba`.
- **debug_draw:** If `true` and `annotation` is `true`, the bounding boxes will be displayed in the image for annotated objects.
- **annotation:** If `true`, the annotation information will be provided in the output data.
- **include_tags:** If `true`, the actor tag information will be included in annotations.
- **include_obb:** If `true`, the actor oriented bounding box information will be included in annotations.
- **cull_partial_frame:** If `true`, the actors that are only partially in frame will be removed from annotations.
- **far_plane:** The maximum distance in centimeters to annotate actors in the scene.
- **desired_tags:** If this array is not empty, the only actors with the tags specified here will be included in annotations.
- **viewport:** if `enable_viewport` is set to `true`, a new window will open with this camera as a viewport camera. For more information see [multi-viewport](../Multi-viewport)
- **color_filter_array:** If `use_cfa` set to `true`, enables color filter array. For more information, see [color filter array (bayer)](./#color-filter-arrays-bayer)
- **ray_tracing_enable:** If set to `true`, enables ray tracing. For more information, see [ray tracing](./#real-time-ray-tracing-tuning)

## Grayscale Camera

Provides a grayscale camera stream with optional bounding boxes for dynamic objects in the scene.


<div class ='multi_img_container'>
<div class="wide_img">

``` json
 {
    "type": "Camera",
    "listen_port": 8012,
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
    "dynamic_range":  50,
    "fov": 60.0,
    "focal_length": 9.0,
    "fstop": 1.4,
    "min_shutter":  0.000500,
    "max_shutter":  0.001400,
    "sensor_size":  9.07,
   "channels" : "gray",
   "annotation": {
      "include_annotation": false,
      "desired_tags": [
        "traffic_sign", "vehicle"
      ],
      "far_plane": 10000.0,
      "include_tags": true,
      "include_obb": false,
      "cull_partial_frame":false,
      "debug_draw":false
    },
    "max_distance":50000.0,
    "channel_depth":1
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
     "listen_port": 8013,
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
    "dynamic_range":  50,
    "fov": 60.0,
    "focal_length": 9.0,
    "fstop": 1.4,
    "min_shutter":  0.000500,
    "max_shutter":  0.001400,
    "sensor_size":  9.07,
    "channels" : "gray",
    "annotation": {
      "include_annotation": false,
      "desired_tags": [
        "traffic_sign", "vehicle"
      ],
      "far_plane": 10000.0,
      "include_tags": true,
      "include_obb": false,
      "cull_partial_frame":false,
      "debug_draw":false
    },
    "max_distance":50000.0,
    "channel_depth": 1
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
      "listen_port": 8014,
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
    "dynamic_range":  50,
    "fov": 60.0,
    "focal_length": 9.0,
    "fstop": 1.4,
    "min_shutter":  0.000500,
    "max_shutter":  0.001400,
    "sensor_size":  9.07,
     "annotation": {
      "include_annotation": false,
      "desired_tags": [
        "traffic_sign", "vehicle"
      ],
      "far_plane": 10000.0,
      "include_tags": true,
      "include_obb": false,
      "cull_partial_frame":false,
      "debug_draw":false
    },
    "max_distance":50000.0,
    "channels":"gray",
    "channel_depth":4
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

## Fisheye and Wide Angle Camera

### Scaramuzza based Fisheye Camera

The fisheye lens model was expanded to include the Poly1FisheyeCamera model. This model is defined by a polynomial (Scaramuzza’s model a0, a2, a3, a4) which describes the projection onto the image sensor. The model can generically describe any camera lens projection but is best used for difficult models such as fisheye and ultra wide angle lenses. The polynomial parameters can be calibrated automatically from a chessboard dataset using [mathwork’s toolbox](https://www.mathworks.com/help/vision/ug/fisheye-calibration-basics.html).

<div class ='multi_img_container'>
<div class="wide_img">

```json
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

### Equidistant Fisheye Camera

```json
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

<p class="img_container">
  <img class="wide_img" src="../img/fisheye1.png" />
</p>


The configuration for a Equidistant Fisheye Camera requires some additional settings.

 - **vignette_bias**: Before the hard mechanical vignette at what point should the fade start normalized with respect to the diameter of the fisheye.
 - **vignette_radius_start**: in the transition to black at the start of the mechanical vignette start with this value of black before the linear fade.
 - **fisheye_pixel_diameter**: To obtain a bounded fisheye set the pixel diameter to the smallest axis, or to the measured real pixel diameter limit from the image. To obtain a diagonal bounded fisheye set the pixel diameter to the hypotenuse of the image. To obtain a fisheye that is greater than the image plane use the value from your model
 - **face_size**: Increasing this number improves image quality and vice versa with diminishing returns with respect to the image resolution. face_size should be smaller than the largest resolution.

## 360 Camera

Provides a 360 view of the scene.

<div class ='multi_img_container'>
<div class="wide_img">

```json
{
    "type":"Camera360",
    "listen_port":8017,
    "location": {
      "x": 0.0,
      "y": 0.0,
      "z": 250.0
    },
    "rotation": {
      "pitch": 0.0,
      "yaw":  0.0,
      "roll": 0.0
    },
    "stream_dimensions": {
      "x": 512.0,
      "y": 512.0
    },
    "face_size": 512,
    "fov": 180.0,
    "viewport": {
        "enable_viewport": false,
        "fullscreen": false,
            "monitor_name": "",
        "monitor_number": 0,
        "window_offset": {
            "x": 32,
            "y": 32
        },
        "window_size": {
            "x": 0,
            "y": 0
        }
    }
}
```
</div>

<div class="img_container">
  <video class='half_screen_img' height=400px muted autoplay loop>
    <source src="https://cdn.monodrive.io/QP-360.mov" type="video/mp4">
  </video>
</div> 

</div>



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

### Annotation

``` json
{
	"actors": [{
			"2d_bounding_boxes": [{
				"2d_bounding_box": [
					158.12020874023438,
					191.5050811767578,
					261.953125,
					272.0334167480469
				],
				"name": "Body"
			}],
			"name": "sedan_monoDrive_02_HD4_128",
			"oriented_bounding_box": [{
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
			}],
			"tags": [
				"vehicle",
				"static",
				"car"
			]
		},
		{
			"2d_bounding_boxes": [{
				"2d_bounding_box": [
					189.89840698242188,
					203.3860321044922,
					260.5026550292969,
					267.6765441894531
				],
				"name": "Body"
			}],
			"name": "subcompact_monoDrive_01_HD_32",
			"oriented_bounding_box": [{
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
			}],
			"tags": [
				"vehicle",
				"static",
				"car"
			]
		}
	],
	"lanes": [{
		"center": [{
			"x": 255.92514038085938,
			"y": 477.407958984375
		}, {
			"x": 255.9377899169922,
			"y": 440.50653076171875
		}],
		"left": [{
			"x": 109.17768859863281,
			"y": 477.3799743652344
		}, {
			"x": 133.64822387695313,
			"y": 440.4832458496094
		}],
		"right": [{
			"x": 402.6728515625,
			"y": 477.435791015625
		}, {
			"x": 378.22747802734375,
			"y": 440.52972412109375
		}],
		"road_id": 0,
		"lane_id": 3,
		"s": 5162.6796875,
		"offset": 0
	}]
}
```

Annotation data comes in an additional JSON message on the same port as the 
camera image data. Each element in the JSON array represents a single annotation 
for a dynamic actor.

- **actors:** Array JSON objects that decribe the actors on the camera frame  
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

- **lanes:** Array JSON objects that decribe the lanes on the camera frame
    - **road_id:** opendrive spec road id, can be used to lookup this lane in the opendrive map.
    - **lane_id:** opendrive lane id, can be be used to identify this lane in the opendrive map.
    - **s:** distance along opendrive lane spline.
    - **offset:** an array of (x,y) points in image space for the left lane marking.
    - **left:** an array of (x,y) points in image space for the left lane marking.
    - **right:** an array of (x,y) points in image space for the right lane marking.
    - **center** an array of (x,y) points in image space for the lane center.

<p class="img_container">
<img class="wide_img" src="../img/lane_annotation.png"/>
</p>

## Camera Coordinate Reference System
A Camera's coordinate reference system is used in relation to Unreal Engine. 

<p class="img_container">
<img class="wide_img" src="../img/camera_reference.png"/>
</p>

## Color Filter Arrays (Bayer)

Real cameras generate RAW images before demosaicing (debayering) into RGB. The monoDrive simulator is able to generate RAW images with any red, green, blue, and clear 2x2 filter pattern at runtime in the monoDrive client API. The demosaicing process on real cameras introduces several different artifacts that your perception system has been trained with and from real camera images. 

To enable color filter, set `use_cfa` to `true`, and select a cfa type such as "rccc" or any combination of r, g, b, or c.

```json
     "color_filter_array": {
          "use_cfa": true,
          "cfa":"rccc"
    }
```

<p class="img_container">
<img class="wide_img" src="../img/RCCC.png"/>
</p>

## Camera as a Viewport Camera

Starting from release 1.12, the user can configure a viewport on any standard, fisheye, or 360 camera. This will open an additional window on the simulation machine to directly display the stream of camera data.

```json
"viewport": {
      "enable_viewport": true,
      "monitor_name": "",
      "monitor_number": 0,
      "fullscreen": false,
      "window_offset": {
        "x": 256.0,
        "y": 256.0
      }
}
```

More information can be found on [Multi-Viewport Page](../Multi-viewport/)

## Real-Time Ray Tracing Tuning

The maps available out of the box in monoDrive have been better tuned for Real-Time Ray Tracing. Real-Time Ray Tracing allows for accurate real-time reflections, better dynamic shadows, and ambient occlusion at the cost of frame rate on lower end machines. The primary viewport also defaults to ray tracing disabled but can be enabled by selecting the post processing volume in the map of interest and turning on ray traced ambient occlusion and ray traced reflections.

For better performance, the camera sensors default to `ray_tracing_enable: false`, but `ray_tracing_enable: true` can be set or added in the configuration at runtime to enable real time ray tracing. 

<p class="img_container">
    <img class='half_lg' src="../img/no_refl1.PNG"/>
    <img class='half_lg' src="../img/refl1.PNG"/>
</p>

<p class="img_container">
    <img class='half_lg' src="../img/no_refl2.PNG"/>
    <img class='half_lg' src="../img/refl2.PNG"/>
</p>

&nbsp;

## Camera Configuration Examples   

### Cull Partial Frame

<p class="img_container">
  <img class='wide_img'src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/configuration/camera/cull_partial_frame.png" />
</p>

### Field Of View Example

<p class="img_container">
  <img class='wide_img'src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/configuration/camera/FOV.png" />
</p>

#### Far Plane Example

<p class="img_container">
  <img class='wide_img'src="https://github.com/monoDriveIO/client/raw/master/WikiPhotos/LV_client/sensors/configuration/camera/far_plane.png" />
</p>