# CarSim

The monoDrive Simulator is run with physx car physics, but can be run with CarSim for more optimal car physics. 

## Prerequisites

1. Build or Download the monoDrive Simulator with newest version of CarSim plugin.
1. Install CarSim 
    - for licensing on a new machine, we will need to reach out to CarSim

## Create CarSim model

1. Open CarsSim
    - accept default licensing and database location

1. Create new dataset
    - File → New Dataset (empty)
    - Category: monoDrive, Title: some title

1. Give simulation control to UE4
    - Uncheck 'Set driver controls here'
    - Uncheck 'Set time step here'
    - Uncheck 'Advanced settings'

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
<!-- Open the Labview closed loop mode example
Edit the simulator configuration
Under 'ego_config' set:
"vehicle_dynamics": "carsim"
 "sim_file": "C:\\Users\\you\\path\\to\\ue4simfile.sim" -->

## Running
1. Start the monoDrive Simulator

1. Click 'Generate Files for this Run' from CarSim

1. Run the configured closed loop example from Labview

1. When done, stop both the labview client and the simulator

1. Go back to CarSim and click 'Video + Plot' to see results



