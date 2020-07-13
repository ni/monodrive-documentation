# CarSim

By default, the monoDrive Simulator uses PhysX for vehicle dynamics. Alternatively, monoDrive also supports co-simulation with CarSim.

## Prerequisites

### monoDrive Simulator
- Windows 10
- monoDrive Scenario Editor
- monoDrive Client (C++, Python or LabVIEW)

### monoDrive CarSim Integration Enabled
- Open `VehicleAI.uproject` with a text editor
- Enable the two required plugins

```json
    {
        "Name": "CarSim",
        "Enabled": true,
        "MarketplaceURL": "com.epicgames.launcher://ue/marketplace/content/2d712649ca864c80812da7b5252f5608"
    },
    {
        "Name": "monoDriveVehiclesCarSim",
        "Enabled": true
    }
```
- Launch the editor and the integration will be built

### CarSim Prerequisites 
- VehicleSim Dynamics plugin for Unreal Engine 4.24
- CarSim 2020.0
    - Minimum licenses:
        - CarSim Browser and Graphical User Interface
        - CarSim Solver for Windows


## Create CarSim model

1. Open CarsSim
    - accept default licensing and database location

1. Create new dataset
    - File → New Dataset (empty)
    - Category: monoDrive, Title: some title

1. Give simulation control to UE4
    - Unselect 'Set driver controls here'
    - Unselect 'Set time step here'
    - Unselect 'Advanced settings'

1. Select vehicle
    - Click drop down under 'Simulated Test Specifications'
        - Click 'Vehicles' → 'Vehicle: Assembly'
    - Select a model (e.g. 'D-Class, Sedan')

1. Visualization
    - Under 'Run from Alternate Directory'
        - select ERD 32-bit 
            - check 'Write all outputs'
    - Under the 'Video' button, open the drop down
        - Click 'Vehicle References'
            - select '13 Azm, 9 El, 18 m, No Yaw'
    - Click the dropdown for 'More plots' and select a number
    - Populate these empty datasets with predefined plots

1. (optional) Update params 
    - Under the selected vehicle, any parameters can be updated as desired

1. Generate simfile
    - Under 'Run from Alternate Directory', open the drop down next to 'models' 
        - select 'Models: Transfer to Local Windows Directory'
    - Under the dataset drop down, select 'UE4 simfile generator'
        - Click 'Duplicate' to make a copy
            - Update the working directory and other fields as desired
            - For example, set the directory to 'C:\Users\devin\dump\carsim'
    - Go back to the main dataset and select the copy for the simfile generator
    - Click 'Generate Files for this Run'
    - 'ue4simfile.sim' should appear in the working directory


## Configure the monoDrive simulator
1.  Open up the closed loop scenario file with a text editor

1.  Find the vehicle with the `ego` tag
    - Update the `class_path` to point at the CarSim vehicle and add a field to specify the `sim_file`. It should read:

```json
    "class_path": "/Game/CarSim/VS_Core/CarSimDemoVehicle.CarSimDemoVehicle_C",
    "sim_file" : "C:\\Users\\you\\path\\to\\ue4simfile.sim",
```

## Running

<div class="img_container">
  <video width=650px height=440px muted controls autoplay loop>
    <source src="https://cdn.monodrive.io/readthedocs/monodrive_carsim_cosimulation.mp4" type="video/mp4">
  </video>
</div>


1. Start the monoDrive Simulator

1. Click 'Generate Files for this Run' from CarSim

1. Run the configured closed loop example from LabVIEW

1. When done, stop both the LabVIEW client and the simulator

1. Go back to CarSim and click 'Video + Plot' to see results
