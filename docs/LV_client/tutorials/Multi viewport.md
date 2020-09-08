# Configure multi viewports

### Multi viewport
Starting from release **1.12** the user can now configure secondary viewports. Modify the `"listen_port"` on the sensor configuration with any non-zero value to create additional viewports. User can modify the location and pose of the secondary viewports.

Note: A value of zero on the `"listen_port"` is reserve for the main viewport. 

<p class="img_container">
<img class="lg_img" src="../viewport.png"/>
</p>

### HUD
The main viewport includes a HUD, showing indicators such as speed, gear, etc. To enable set to true the tag `use_vehicle_hud` on the sensor configuration. The HUD can only be configured on the main port.

<p class="img_container">
<img class="lg_img" src="../hud.png"/>
</p>

### Using the LabVIEW Client
When using the LabVIEW Client, the user needs to create additional instances of the `mono_viewport.vi` and change the `"listen_port"` and location and pose if desired. Then the user needs to add them to the main application as follows.

<p class="img_container">
<img class="lg_img" src="../lv_viewport.png"/>
</p>