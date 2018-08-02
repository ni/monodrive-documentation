#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import logging

import time

from monodrive import SimulatorConfiguration, VehicleConfiguration
from monodrive.vehicles import SimpleVehicle
from monodrive.vehicles import TeleportVehicle
from monodrive import Simulator

ManualDriveMode = True

if __name__ == "__main__":
        
    # Simulator configuration defines network addresses for connecting to the simulator and material properties
    simulator_config = SimulatorConfiguration('simulator.json')
    
    # Vehicle configuration defines ego vehicle configuration and the individual sensors configurations
    vehicle_configuration = VehicleConfiguration('test.json')

    simulator = Simulator(simulator_config)
    simulator.send_simulator_configuration()
    b_running = True
    while b_running:
        simulator.restart_event.clear()
        simulator.send_vehicle_configuration(vehicle_configuration)

        # Start Vehicle
        if ManualDriveMode == True:
            ego_vehicle = simulator.start_vehicle(vehicle_configuration, TeleportVehicle)
        else:
            ego_vehicle = simulator.start_vehicle(vehicle_configuration, SimpleVehicle)

        logging.getLogger("simulator").info('Starting vehicle')
        ego_vehicle.start()

        # Waits for the restart event to be set in the control process
        simulator.restart_event.wait()

        # Terminates vehicle and sensor processes
        simulator.stop()
        b_running = False
        logging.getLogger("simulator").info("episode complete")
        time.sleep(5)




