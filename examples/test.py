#!/usr/bin/env python

from monodrive import SimulatorConfiguration, VehicleConfiguration
from monodrive.vehicles import SimpleVehicle
from monodrive import Simulator

if __name__ == "__main__":

    # Simulator configuration defines network addresses for connecting to the simulator and material properties
    simulator_config = SimulatorConfiguration('simulator.json')

    # Vehicle configuration defines ego vehicle configuration and the individual sensors configurations
    vehicle_config = VehicleConfiguration('demo.json')
    #vehicle_config = VehicleConfiguration('light.json')

    simulator = Simulator(simulator_config)

    while True:
        # Start Vehicle
        simulator.start_vehicle(vehicle_config, SimpleVehicle)

        # Waits for the restart event to be set in the control process
        simulator.restart_event.wait()

        # Terminates vehicle and sensor processes
        simulator.stop()
