# PhysX Vehicles

Unreal Engine uses PhysX for the vehicle physics model which monoDrive's Traffic vehicles inherit from. Through Unreal Engine, users will be able to customize the mechanical settings of PhysX vehicles in the monoDrive Scenario Editor and Simulator.

To see details of the PhysX model, see [PhysX documentation](https://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/guide/Manual/Vehicles.html).

## Suspension, Wheel, and Tire Configuration

To customize the suspension, wheel or tire properties, create a wheel class that is a child blueprint child of the VehicleAIWheelFront(and Rear). 
    
1. Navigate to the VehicleAIWheel either directly in the Content browser or by selecting the magnifying glass under Vehicle Setup -> Wheel Setups in any of the VehicleMovement components attached to any of the blueprint vehicles in the Vehicles folder.

    <p class="img_container">
    <img class="wide_img" src="../img/wheel_config1.png" />
    </p>

1. After sub-classing the wheel class, the wheel with settings can be customized. This includes all the suspension settings in PhysX, tire stiffness and friction properties, mass, collision geometry, and more.

    <p class="img_container">
    <img class="wide_img" src="../img/wheel_config2.png" />
    </p>

To set a custom tire configuration which contains the tires friction parameter (note that friction is also set on the materials in the environment and scaled by this number), right click in the Content Browser, and then select Miscellaneous -> Data Asset and search for TireConfig. This will create a custom tire configuration that can then be selected in the wheel blueprint. Note that the default value is 1 which means that all friction properties will be derived by the environment’s materials. Changing this property is only recommended for advanced users, the friction is multiplicative with the environment surface’s frictional value.

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