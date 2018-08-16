#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"


import logging
import time

try:
    import prctl
except:
    pass

from monodrive import SimulatorConfiguration, VehicleConfiguration, Simulator
from monodrive.ui import GUI
from monodrive.vehicles import SimpleVehicle
from monodrive.vehicles import TeleportVehicle


ManualDriveMode = False

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

    #prctl.set_proctitle("monoDrive")

    gui = None
    while episodes > 0:
        simulator.restart_event.clear()
        simulator.send_vehicle_configuration(vehicle_configuration)
        logging.getLogger("simulator").info('Starting vehicle')
        ego_vehicle.start()

        gui = GUI(ego_vehicle, settings=simulator_config.gui_settings)

        # time.sleep(30)
        # simulator.restart_event.set()
        # Waits for the restart event to be set in the control process
        simulator.restart_event.wait()

        # gui_multi_proc.stop()
        # gui.stop()

        # Terminates vehicle and sensor processes
        simulator.stop()

        logging.getLogger("simulator").info("episode complete")
        time.sleep(5)

        episodes = episodes - 1

    if gui is not None:
        gui.stop()

    logging.getLogger("simulator").info("Good Bye!")



