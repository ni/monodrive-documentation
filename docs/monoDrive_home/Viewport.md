# Viewport

The Viewport sensor allows a client to configure the camera that is displayed in 
the monoDrive Simulator or Scenario Editor. 

To add an additional Viewport, see [Multi Viewport](../Multi-viewport)

## Configuration

``` json
{
 "type": "ViewportCamera",
 "listen_port": 0,
 "server_ip": "127.0.0.1",
 "server_port":8999,
 "description": "",
 "use_vehicle_hud": false,
 "location": {
   "x":  -800.0,
   "y": 0.0,
   "z": 400.0
   },
 "rotation": {
   "pitch": -15.0,
   "yaw": 0.0,
   "roll": 0.0
   },
 "fov": 60.0
}

```
- **listen_port:** Any non-zero value will create a secondary viewport. A value of zero is reserved for the main viewport.      
- **use_vehicle_hud:** enables the HUD indicators such as speed, gear, etc. to launch any secondary viewport.   
- **fov:** Sets the horizontal angle of view of the viewport camera.   


### Configuration Examples  

On the left of the image, there are 3 secondary viewport and on the right image there is the main viewport with HUD enable on the monoDrive Editor.

<p class="img_container">
  <img class="wide_img" src="https://github.com/monoDriveIO/documentation/raw/master/WikiPhotos/LV_client/sensors/configuration/viewport/viewport.png" />
</p>
