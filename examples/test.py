#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import logging

import time

from monodrive import SimulatorConfiguration, VehicleConfiguration, Simulator
from monodrive.gui import GuiMultiProcess
from monodrive.ui import GUI
from monodrive.vehicles import SimpleVehicle
from monodrive.vehicles import TeleportVehicle

ManualDriveMode = True


if __name__ == "__main__":
        
    # Simulator configuration defines network addresses for connecting to the simulator and material properties
    simulator_config = SimulatorConfiguration('simulator.json')
    
    # Vehicle configuration defines ego vehicle configuration and the individual sensors configurations
    vehicle_configuration = VehicleConfiguration('test.json')

    simulator = Simulator(simulator_config)
    simulator.send_simulator_configuration()

    episodes = 1  # TODO... this should come from the scenario config
    # Setup Ego Vehicle
    if ManualDriveMode == True:
        ego_vehicle = simulator.get_ego_vehicle(vehicle_configuration, TeleportVehicle)
    else:
        ego_vehicle = simulator.get_ego_vehicle(vehicle_configuration, SimpleVehicle)

    gui = GUI(ego_vehicle)

    while episodes > 0:
        simulator.restart_event.clear()
        simulator.init_episode(vehicle_configuration)
        logging.getLogger("simulator").info('Starting vehicle')
        ego_vehicle.start()

        #gui_multi_proc = GuiMultiProcess(ego_vehicle)
        #gui_multi_proc.start()
        time.sleep(20)
        simulator.restart_event.set()
        # Waits for the restart event to be set in the control process
        simulator.restart_event.wait()

        #gui_multi_proc.stop()
        #gui.stop()

        # Terminates vehicle and sensor processes
        simulator.stop()

        logging.getLogger("simulator").info("episode complete")
        time.sleep(5)

        episodes = episodes - 1

gui.stop()
logging.getLogger("simulator").info("Good Bye!")



