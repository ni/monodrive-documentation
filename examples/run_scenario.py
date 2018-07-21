#!/usr/bin/env python
import os

from monodrive import SimulatorConfiguration
from monodrive.constants import BASE_PATH
from monodrive.vehicles import SimpleVehicle
from monodrive import Simulator
from monodrive.scenario import Scenario


if __name__ == "__main__":

    # Simulator configuration defines network addresses for connecting to the simulator and material properties
    simulator_config = SimulatorConfiguration('simulator.json')

    scenario = Scenario('MDLaneChanger.xosc')

    simulator = Simulator(simulator_config)

    while True:

        # Start Scenario
        simulator.start_scenario(scenario, SimpleVehicle)

        # Waits for the restart event to be set in the control process
        simulator.restart_event.wait()

        # Terminates vehicle and sensor processes
        simulator.stop()


