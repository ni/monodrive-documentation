#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"


import logging
import sys
import time

try:
    import prctl
except:
    pass

from monodrive import SimulatorConfiguration, VehicleConfiguration, Simulator
from monodrive.ui import GUI
from monodrive.util import InterruptHelper
from monodrive.vehicles import SimpleVehicle
from monodrive.vehicles import TeleportVehicle

from monodrive.networking.client import Client

ManualDriveMode = True




if __name__ == "__main__":

    # Simulator configuration defines network addresses for connecting to the simulator and material properties
    simulator_config = SimulatorConfiguration('simulator.json')

    # Vehicle configuration defines ego vehicle configuration and the individual sensors configurations
    vehicle_config = VehicleConfiguration('demo.json')

    client = Client((simulator_config.configuration["server_ip"],
                     simulator_config.configuration["server_port"]))

    if not client.isconnected():
        client.connect()

    simulator = Simulator(client, simulator_config)
    simulator.send_configuration()

    # time.sleep(30)

    episodes = 1  # TODO... this should come from the scenario config
    # Setup Ego Vehicle

    from monodrive import VehicleConfiguration
    from monodrive.vehicles import LV_Vehicle

    simulator.send_configuration()
    map_data = simulator.request_map()

    # Setup Ego Vehicle
    if ManualDriveMode == True:
        ego_vehicle = TeleportVehicle(simulator_config, vehicle_config, map_data)
    else:
        ego_vehicle = SimpleVehicle(simulator_config, vehicle_config, map_data)


    # prctl.set_proctitle("monoDrive")
    #
    gui = None


    while episodes > 0:
        helper = InterruptHelper()

        simulator.restart_event.clear()
        simulator.send_vehicle_configuration(vehicle_config)
        logging.getLogger("simulator").info('Starting vehicle')
        ego_vehicle.update_fmcw_in_config()
        ego_vehicle.start_sensor_streaming(client)
        ego_vehicle.start_sensor_listening()

        gui = GUI(ego_vehicle, simulator)

        ego_vehicle.init_vehicle_loop(client)

        # Waits for the restart event to be set in the control process
        # time.sleep(100)
        helper.wait(simulator.restart_event)
        simulator.restart_event.wait()

        gui.stop()

        # Terminates vehicle and sensor processes
        simulator.stop()

        logging.getLogger("simulator").info("episode complete")
        time.sleep(1)

        episodes = episodes - 1

    logging.getLogger("simulator").info("Good Bye!")


