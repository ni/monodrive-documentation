### Multi viewport
Starting from release **1.12**, the user can configure secondary viewports on any standard , fisheye or 360 camera. This is primarily used on applications like HIL, real time systems, direct injection, etc.

### Usage
User should add the following json tags to the sensor configuration:

```
"viewport": {
        "enable_viewport": false,
        "fullscreen": false,
        "monitor_name": "",
        "monitor_number": -1,
        "window_offset": {
            "x": 0,
            "y": 0
        },
        "window_size": {
            "x": 0,
            "y": 0
        }
}
```
Set `"enable_viewport"` to true to display a secondary viewport. 

## Example
<p class="img_container">
<img class="lg_img" src="../img/multiviewport.png"/>
</p>


