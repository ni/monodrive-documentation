# Vehicles
The monoDrive Simulator provides users with the ability to customize color and the model for the ego vehicle as well as for the vehicles that interact with it on simulation. The user may specify the initial conditions for any car on the scene, i.e. initial pose and initial velocity.

## Change the vehicle's model
<div class="img_container">
  <video width=650px height=440px muted controls autoplay loop>
    <source src="http://cdn.monodrive.io/readthedocs/vehicles.mp4" type="video/mp4">
  </video>
</div>

**Current Blueprints available**   
The current models available can be find on the  `Content Browser` under `Vehicles`

When selecting a model programmatically (i.e. using any client) the user can assign the model for the ego vehicle randomly from the vehicle blueprints. This can be found in the Editor folder:  
`Config/ConfigRandomVehicleSettings.ini`   

```
[VehicleBlueprints]
VehicleBlueprint=Blueprint'/Game/Vehicles/sedan_monoDrive_01.sedan_monoDrive_01_C'
VehicleBlueprint=Blueprint'/Game/Vehicles/sedan_monoDrive_02.sedan_monoDrive_02_C'
VehicleBlueprint=Blueprint'/Game/Vehicles/SUV_monoDrive_01.SUV_monoDrive_01_C'
VehicleBlueprint=Blueprint'/Game/Vehicles/truck_monoDrive_01.truck_monoDrive_01_C'
VehicleBlueprint=Blueprint'/Game/Vehicles/subcompact_monoDrive_01.subcompact_monoDrive_01_C'
VehicleBlueprint=Blueprint'/Game/Vehicles/minivan_monoDrive_01.minivan_monoDrive_01_C'
VehicleBlueprint=Blueprint'/Game/Vehicles/coupe_monoDrive_01.coupe_monoDrive_01_C'
VehicleBlueprint=Blueprint'/Game/Vehicles/crossover_monoDrive_01.crossover_monoDrive_01_C'
VehicleBlueprint=Blueprint'/Game/Vehicles/compact_monoDrive_01.compact_monoDrive_01_C'
```
**Note:** To add a customized vehicle model, add the path to the blueprint in this file.

## Change the vehicle's color
<div class="img_container">
    <video width=650px height=340px muted controls autoplay loop>
        <source src="http://cdn.monodrive.io/readthedocs/vehicle_color.mp4" type="video/mp4">
    </video>
</div> 

**Current colors available**   
From the Content Bowser under `Vehicles` and `CarPaint`, the user has control over the current colors available for all car models.

```
Carpaint_BabyBlue
Carpaint_Beige
Carpaint_Black
Carpaint_Copper
Carpaint_DarkGrey
Carpaint_Gold
Carpaint_LightBlue
Carpaint_NavyBlue
Carpaint_OliveGreen
Carpaint_Red
Carpaint_Silver
Carpaint_White
```
When selecting a color programmatically (i.e. using any client) the user can assign the color to use for the ego vehicle using the `vehicle.json` with the `body` tag. 
```
"body": {
    "color": "Carpaint_White.Carpaint_White",
    "type": "/Game/Vehicles/crossover_monoDrive_01.crossover_monoDrive_01_C"
  }
```
## PID Speed Controllers
### Speed Maintain PID
Each vehicle has a pre-defined PID parameters to control how closely and accurate maintains its own desired speed.   

### Follower PID
Each vehicle has a pre-defined PID parameters to control how closely and accurate follow other vehicles. The user can tune this parameters.

## Vehicle Dynamics
monoDrive Scenario Vehicles are built on PhysX vehicles which are customizable out of the box. All of the PhysX mechanical settings can be cusomized in the editor to match the user's vehicle such as the suspension, drivetrain type, transmission gearing, suspension parameters, and much more.

<div class="img_container">
    <video width=650px height=340px muted controls autoplay loop>
        <source src="http://cdn.monodrive.io/readthedocs/vehicle_mechanical.mp4" type="video/mp4">
    </video>
</div> 

## Vehicle Axis and Rotations
### Axis
The vehicle axis are defined as follows:   
**X:** Points to the front of the car.   
**Y:** Points to the right of the car.   
**Z:** Points to the top of the car.   

<div class="img_container">
    <video width=650px height=340px muted controls autoplay loop>
        <source src="http://cdn.monodrive.io/readthedocs/axis.mp4" type="video/mp4">
    </video>
</div>   

### Rotation
**Yaw:** Positive angle when rotating clockwise around the Z axis and negative counterclockwise.   
**Pitch:** Negative angle when rotating clockwise around the Y axis and positive counterclockwise.   
**Roll:** Positive angle when rotating clockwise around the X axis and negative counterclockwise.   

<div class="img_container">
    <video width=650px height=340px muted controls autoplay loop>
        <source src="http://cdn.monodrive.io/readthedocs/vehicles_yaw_pitch_roll.mp4" type="video/mp4">
    </video>
</div> 

## Initial Conditions
When configuring the ego vehicle using the C++ client or the Python client. The user can assign the initial conditions for the ego vehicle:   

**position**: Initial position on the map on the x,y,z axis.   
```json
"position": [
  0.0,
  0.0,
  0.0
  ]
```   
**orientation**: On quaternion form (euler angles) with respect to the vehicle axis.   
```json
"orientation": [
    0.0,
    0.0,
    0.0,
    1.0
  ]
```    
In addition, the user can provide **yaw**, **pitch** and **roll** form (in degrees) with respect to the vehicle axis, using the following notation.   
```json
"orientation": {
  "yaw":0.0,
  "pitch":0.0,
  "roll":0.0
}
```    
**velocity**: Linear velocity on cm/second.
```json
"velocity": [
    0.0,
    0.0,
    0.0
  ]
```
**angular_velocity**: Angular velocity in x, y and z axis. Expressed on radians/s.   
```json
"angular_velocity": [
  0.0,
  0.0,
  0.0
]
```     
## Vehicle Configuration
The following configuration is an example of how to create a `vehicle.json` to use with the C++ client and the Python client. 

```json
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

### Configuration Tags
- **vehicle_dynamics**: Physics engine to use to perform vehicle dynamics.
- **tags**: Describe the vehicle for classification or filtering purposes.
