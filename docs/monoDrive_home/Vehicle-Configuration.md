# Scenario Vehicle
The monoDrive Simulator provides users with the ability to customize the color and model to use for the ego vehicle as well as for the vehicles that interact with it on simulation. The simulator also provides users with the ability to specify the initial conditions for any car on the scene, i.e. initial pose and initial velocity. 

## Change the vehicle's model
<div class="img_container">
  <video width=650px height=480px muted controls autoplay loop>
    <source src="http://cdn.monodrive.io/readthedocs/vehicles.mp4" type="video/mp4">
  </video>
</div>
**Current Blueprints available**   
On the monoDrive Editor look for the Content Browser, from the folders select Vehicles. You will find the current models available.

When selecting a model programmatically (i.e. using any client) you can assign the model to use for the ego vehicle to be random from the vehicle blueprints, you can find it on your Editor folder under:  
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
**Note:** If you want to add your own vehicle model, add the path to the blueprint to this file.  

## Change the vehicle's color
<div class="img_container">
    <video width=650px height=340px muted controls autoplay loop>
        <source src="http://cdn.monodrive.io/readthedocs/vehicle_color.mp4" type="video/mp4">
    </video>
</div> 

**Current colors available**   
On the monoDrive Editor look for the Content Browser. From the folders select Vehicles and then select CarPaint, you will find the current colors availables for any car.

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
When selecting a color programmatically (i.e. using any client) you can assign the color to use for the ego vehicle using the `vehicle.json` with the `body` tag. 
```
"body": {
    "color": "Carpaint_White.Carpaint_White",
    "type": "/Game/Vehicles/crossover_monoDrive_01.crossover_monoDrive_01_C"
  }
```
## PID Speed Controllers
### Follower PID
Each vehicle has a pre-defined PID parameters to control how closely and accurate follow others vehicles. You can tune this parameters.

### Speed Maintain PID
Each vehicle has a pre-defined PID parameters to control how closely and accurate maintains the desired speed.

## Vehicle Dynamics
Using the monoDrive vehicles you have access to the Mechanical Setup to control the drivetrain type (4WD,FDW,RDW,etc), the maximum RPM and damping, the transmission's type, as well of the gear ratio if not automatic transmission, you can also set suspension settings and more.

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
When configuring the ego vehicle using the C++ client or the Python client. You can assign the initial conditions for the ego vehicle:   

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
You can provide the orientation in **yaw**, **pitch** and **roll** form (degrees) with respect to the vehicle axis, using the following notation.   
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
