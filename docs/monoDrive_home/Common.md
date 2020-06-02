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
}
```

## Configuration

- **type:** The values of type can be: `Camera`, `Lidar`, `IMU`, etc. depending on what type of sensor being configured.
- **listen_port:** The TCP or UDP port used by the simulator to transmit sensor data.
- **location:**: The location of the sensor in centimeters relative to the EGO vehicle's origin to place a sensor.
    - **x:** The x position
    - **y:** The y position
    - **z:** The z position
- **rotation**: The rotation of the sensor, in degrees, relative to the orientation of the EGO vehicle.
    - **pitch:** The pitch angle
    - **yaw:** The yaw angle
    - **roll:** The roll angle
