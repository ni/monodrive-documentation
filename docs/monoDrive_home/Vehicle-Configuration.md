# Vehicles

The monoDrive Simulator provides users with the ability to customize color and model for all vehicles in the simulation. The user may specify the initial conditions for any car in the scene, i.e. initial pose and initial velocity.

## Add Vehicle Model

For information on adding a new vehicle model contact monoDrive support at support@monodrive.io

## Change the Vehicle's Model
<div class="img_container">
  <video width=650px height=440px muted controls autoplay loop>
    <source src="https://cdn.monodrive.io/readthedocs/vehicles.mp4" type="video/mp4">
  </video>
</div>

**Current Blueprints available**   

The current models available can be find in  `Content Browser` folder under `Vehicles`.


When selecting a model programmatically (i.e. using any client) the user can assign the model for the ego vehicle randomly from the vehicle blueprints. This can be found in the Editor folder:  
`Config/ConfigRandomVehicleSettings.ini`   

```
[PhysXVehicles]
PhysXVehicle=Blueprint'/Game/Vehicles/sedan_monoDrive_01.sedan_monoDrive_01_C'
PhysXVehicle=Blueprint'/Game/Vehicles/sedan_monoDrive_02.sedan_monoDrive_02_C'
PhysXVehicle=Blueprint'/Game/Vehicles/SUV_monoDrive_01.SUV_monoDrive_01_C'
PhysXVehicle=Blueprint'/Game/Vehicles/truck_monoDrive_01.truck_monoDrive_01_C'
PhysXVehicle=Blueprint'/Game/Vehicles/subcompact_monoDrive_01.subcompact_monoDrive_01_C'
PhysXVehicle=Blueprint'/Game/Vehicles/minivan_monoDrive_01.minivan_monoDrive_01_C'
PhysXVehicle=Blueprint'/Game/Vehicles/coupe_monoDrive_01.coupe_monoDrive_01_C'
PhysXVehicle=Blueprint'/Game/Vehicles/crossover_monoDrive_01.crossover_monoDrive_01_C'
PhysXVehicle=Blueprint'/Game/Vehicles/compact_monoDrive_01.compact_monoDrive_01_C'
[Vehicles]
[ReplayVehicles]
ReplayVehicle=Blueprint'/Game/Vehicles/sedan_simple_01.sedan_simple_01_C'
ReplayVehicle=Blueprint'/Game/Vehicles/sedan_simple_02.sedan_simple_02_C'
ReplayVehicle=Blueprint'/Game/Vehicles/SUV_simple_01.SUV_simple_01_C'
ReplayVehicle=Blueprint'/Game/Vehicles/truck_simple_01.truck_simple_01_C'
ReplayVehicle=Blueprint'/Game/Vehicles/subcompact_simple_01.subcompact_simple_01_C'
ReplayVehicle=Blueprint'/Game/Vehicles/minivan_simple_01.minivan_simple_01_C'
ReplayVehicle=Blueprint'/Game/Vehicles/coupe_simple_01.coupe_simple_01_C'
ReplayVehicle=Blueprint'/Game/Vehicles/crossover_simple_01.crossover_simple_01_C'
ReplayVehicle=Blueprint'/Game/Vehicles/compact_simple_01.compact_simple_01_C'
```
**Note:** To add a new vehicle model, add the path to the blueprint in this file. 
If adding a vehicle derived from AVehicle class add to the [Vehicles] list and if derived from 
APhysXVehicle add to the [PhysXVehicles] list. If the vehicle is capable of replay add to the 
[ReplayVehicles] list. If you do not wish for a specific vehicle to be chosen for random
 spawning remove it from the lists.

## Change the Vehicle's Color
<div class="img_container">
    <video width=650px height=340px muted controls autoplay loop>
        <source src="https://cdn.monodrive.io/readthedocs/vehicle_color.mp4" type="video/mp4">
    </video>
</div> 

**Current colors available**   
From the `Content Bowser` under `Vehicles` and `CarPaint`, the user has control over the current colors available for all car models.

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

### Closed Loop Mode

When selecting a color programmatically (i.e. using any client) in Closed loop mode, the user can assign a color for the ego vehicle using the scenario file.

```
"body_color": "Carpaint_OliveGreen",
"class_path": "/Game/Vehicles/sedan_monoDrive_01.sedan_monoDrive_01_C"
```

### Replay Mode

When selecting a color programmatically (i.e. using any client) in Replay mode, the user can assign a color for the ego vehicle using the trajectory file.

```
"color": "Carpaint_DarkGrey",
"replay_class": "/Game/Vehicles/compact_simple_01.compact_simple_01_C",
```

## PID Speed Controllers
### Speed Maintain PID
Each vehicle has a pre-defined PID parameters to control how closely and accurately it maintains a desired speed.   

### Follower PID
Each vehicle has a pre-defined PID parameters to control how closely and accurate follow other vehicles. The user can tune these parameters.

## Vehicle Dynamics
monoDrive Scenario Vehicles are built on PhysX vehicles which are customizable out of the box. All of the PhysX mechanical settings can be customized in the editor to match the user's vehicle such as the suspension, drivetrain type, transmission gearing, suspension parameters, and much more.

<div class="img_container">
    <video width=650px height=340px muted controls autoplay loop>
        <source src="https://cdn.monodrive.io/readthedocs/vehicle_mechanical.mp4" type="video/mp4">
    </video>
</div> 

monoDrive also supports co-simulation with CarSim for vehicle dynamics. For more information, please see [CarSim tutorial](CarSim.md).

## Vehicle Axis and Rotations
### Axis
The vehicle axis are defined as follows:   
**X:** Points to the front of the car.   
**Y:** Points to the right of the car.   
**Z:** Points to the top of the car.   

<div class="img_container">
    <video width=650px height=340px muted controls autoplay loop>
        <source src="https://cdn.monodrive.io/readthedocs/axis.mp4" type="video/mp4">
    </video>
</div>   

### Rotation
**Yaw:** Positive angle when rotating clockwise around the Z axis and negative counterclockwise.   
**Pitch:** Negative angle when rotating clockwise around the Y axis and positive counterclockwise.   
**Roll:** Positive angle when rotating clockwise around the X axis and negative counterclockwise.   

<div class="img_container">
    <video width=650px height=340px muted controls autoplay loop>
        <source src="https://cdn.monodrive.io/readthedocs/vehicles_yaw_pitch_roll.mp4" type="video/mp4">
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
**orientation**: Quaternion form.  
```json
"orientation": [
    0.0,
    0.0,
    0.0,
    1.0
  ]
```    
Alternatively, the user can provide Euler Angles, **yaw**, **pitch** and **roll** (in degrees), using the following notation.   
```json
"orientation": {
  "yaw":0.0,
  "pitch":0.0,
  "roll":0.0
}
```    
**velocity**: Linear velocity in cm/second.
```json
"velocity": [
    0.0,
    0.0,
    0.0
  ]
```

**angular_velocity**: Angular velocity on x, y and z axis. Expressed in radians/s.   

```json
"angular_velocity": [
  0.0,
  0.0,
  0.0
]
```     