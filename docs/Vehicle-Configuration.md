# Vehicle Configuration

## Vehicle
The monoDrive Simulator allows you to customize multiple visual and dynamic settings on the ego vehicle.

## Configuration

```
{
  "vehicle_dynamics": "physx",
  "body": {
    "color": "Carpaint_White.Carpaint_White",
    "type": "/Game/Vehicles/crossover_monoDrive_01.crossover_monoDrive_01_C"
  },
  "angular_velocity": [
    0.0,
    0.0,
    0.0
  ],
  "orientation": [
    0.0,
    0.0,
    0.0,
    1.0
  ],
  "position": [
    0.0,
    0.0,
    0.0
  ],
  "tags": [
    "vehicle",
    "dynamic",
    "ego"
  ],
  "velocity": [
    0.0,
    0.0,
    0.0
  ],
  "wheel_speed": [
    0.0,
    0.0,
    0.0,
    0.0
  ],
  "wheels": [
    {
      "id": 1,
      "orientation": [
        0.0,
        0.0,
        0.0,
        1.0
      ]
    },
    {
      "id": 2,
      "orientation": [
        0.0,
        0.0,
        0.0,
        1.0
      ]
    }
  ]
}
```
<p>&nbsp;</p>

## Configuration

### Configuration Tags
- **vehicle_dynamics**: Physics engine to use to perform vehicle dynamics.
- **body**: Visual settings. If not present, the simulator will assign one at random.
  - **color**: Path to the blueprint defining the color of the car to use.
  - **type**: Path to the blueprint defining the model of the car to use.
- **angular_velocity**: 
- **orientation**: Initial, yaw, pitch and roll in degrees.
- **position**: Initial position on the map on the x,y,z axis.
- **tags**: Describe the vehicle for classification purposes or targeting. Usually *["vehicle","dynamic","ego"]*. 
- **velocity**: Linear velocity on cm/second.