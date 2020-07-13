# PhysX Vehicles

Unreal Engine uses PhysX for the vehicle physics model which monoDrive's Traffic vehicles inherit from. Through Unreal Engine, users will be able to customize the mechanical settings of PhysX vehicles in the monoDrive Scenario Editor and Simulator.

To see details of the PhysX model, see [PhysX documentation](https://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/guide/Manual/Vehicles.html).

## Suspension, Wheel, and Tire Configuration

To customize the suspension, wheel or tire properties, create a wheel class that is a child blueprint child of the VehicleAIWheelFront(and Rear). 
    
1. Navigate to the VehicleAIWheel either directly in the Content browser or by selecting the magnifying glass under Vehicle Setup -> Wheel Setups in any of the VehicleMovement components attached to any of the blueprint vehicles in the Vehicles folder.

    <p class="img_container">
    <img class="wide_img" src="../img/wheel_config1.png" />
    </p>

1. This includes all the PhysX suspension settings, tire stiffness, friction properties, mass, collision geometry, and more.

    <p class="img_container">
    <img class="wide_img" src="../img/wheel_config2.png" />
    </p>

To set a custom tire configuration that contains the tires friction parameter (NOTE: that friction is also set on the materials in the environment and scaled by this number), right click in the Content Browser, select Miscellaneous -> Data Asset, and search for TireConfig. This will create a custom tire configuration that can then be selected in the wheel blueprint. Note that the default value is 1 meaning all friction properties will be derived by the environment’s materials. The friction is multiplicative with the environment surface’s frictional value, so be cautious when changing this value because this combination can make the wheel and vehicle's physics unstable and unpredictable if not set correctly.

<p class="img_container">
<img class="wide_img" src="../img/wheel_config3.png" />
</p>

## Mass and Inertial Configuration

Mass and inertial properties are set under the Physics tab inside the vehicles Skeletal Mesh.

<p class="img_container">
<img class="wide_img" src="../img/mass_config.png" />
</p>

## Gear, Transmission, and Engine Configuration

Gear, Transmission and Engine properties can be set under Mechanical Setup on the VehicleMovement component.

<p class="img_container">
<img class="wide_img" src="../img/engine_config.png" />
</p>

## Steering Curve

The Steering Curve enables the user to limit the vehicle’s maximum turn angle as a function of velocity.

<p class="img_container">
<img class="wide_img" src="../img/steering_config.png" />
</p>