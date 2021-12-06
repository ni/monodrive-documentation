# Common

Every monoDrive sensor shares the following format for common sensor properties. 
For additional parameters descriptions of specific sensors, see the sensor's 
configuration documentation.

```json
{
  "type": string,
  "id": string,
  "listen_port": int,
  "location": {
    "x": float,
    "y": float,
    "z": float
  },
  "rotation": {
    "pitch": float,
    "yaw": float,
    "roll": float
  }
  "use_parent_component_attachment": bool,
  "parent_component_attachment": string,
  "use_attach_socket": bool,
  "attach_socket": string
}
```

## Configuration

- **type:** The values of type can be: `Camera`, `Lidar`, `IMU`, etc. depending on what type of sensor being configured.
- **listen_port:** The TCP or UDP port used by the simulator to transmit sensor data.
- **location:**: The location of the sensor in centimeters relative to the ego vehicle's origin to place a sensor.
    - **x:** The x position
    - **y:** The y position
    - **z:** The z position
- **rotation**: The rotation of the sensor, in degrees, relative to the orientation of the ego vehicle.
    - **pitch:** The pitch angle
    - **yaw:** The yaw angle
    - **roll:** The roll angle
- **Attachment** Attaching the sensor allows you to place a sensor at the origin of a particular component, or on a socket defined by the actor's blueprint. This allows sensor placement to be preserved in .uasset's, and allows for placement and rotation to be defined in the editor. **Note** location and rotation above will transform the sensor relative to the socket/component, and are not necessary if attachment is specified.
  - **use_parent_component_attachment** Specifies whether we should attempt to attach the sensor to a particular component of the ego vehicle.
  - **parent_component_attachment** String matching the component of the ego actor where we would like to attach the sensor.
  - **use_attach_socket** Specifies whether we should attempt to attach the sensor to a particular socket subcomponent of the parent component.
  - **attach_socket** String matching the name of the socket to which we would like to attach the sensor. 