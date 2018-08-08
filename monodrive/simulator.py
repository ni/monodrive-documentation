
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import logging
from logging.handlers import RotatingFileHandler

from multiprocessing import Event, Process, Queue
import threading
#import os, psutil  # for removing processing after episode
#try:

import sys

from monodrive.networking import messaging
from monodrive.networking.client import Client
from monodrive.constants import *
from monodrive.scene import Map

from monodrive import VehicleConfiguration


class Simulator(object):

    def __init__(self, simulator_configuration):
        self.simulator_configuration = simulator_configuration
        self.restart_event = Event()
        self.ego_vehicle = None
        self.scenario = None
        self.map = None
        self._client = None
        self.setup_logger()

    def start_scenario(self, scenario, vehicle_class):
        self.scenario = scenario

        # Send both simulator configuration and scenario
        # self.send_simulator_configuration(self.simulator_configuration)
        self.send_scenario(scenario)
        print('Received Response From Sending Scenario')

        # Get Ego vehicle configuration from scenario, use that to create vehicle process
        vehicle_configuration = scenario.ego_vehicle_config
        vehicle_configuration = VehicleConfiguration.init_from_json(vehicle_configuration.to_json)
        self.ego_vehicle = vehicle_class(self, vehicle_configuration, self.restart_event)

        # Start the Vehicle process
        self.ego_vehicle.start_scenario(scenario)

    def start_vehicle(self, vehicle_configuration, vehicle_class):
        # Create vehicle process form received class
        self.ego_vehicle = vehicle_class(self, vehicle_configuration, self.restart_event, self.map_data)
        return self.ego_vehicle

    def stop(self):

        # Stop all processes
        logging.getLogger("simulator").info("start shutting down simulator client")
        self.ego_vehicle.stop()
        logging.getLogger("simulator").info("simulator client shutdown complete")

        ## get the pid of this program
        #pid=os.getpid()

        ## when you want to kill everything, including this program
        #self.kill_process_tree(pid, False)

                # Disconnect from server
        #self.client.disconnect()
        #self.client.stop()

        self.map.stop()

    def kill_process_tree(self, pid, including_parent=True):
        parent = psutil.Process(pid)
        for child in parent.children(recursive=True):
            print("kill monodrive child {0}".format(child))
            child.kill()

        if including_parent:
            parent.kill()


    @property
    def client(self):
        if self._client is None:
            self._client = Client((self.simulator_configuration.server_ip,
                                   self.simulator_configuration.server_port))

        if not self._client.isconnected():
            self._client.connect()
        return self._client

    def request(self, message_cls, timeout=5):
        return self.client.request(message_cls, timeout)

    def request_sensor_stream(self, message_cls):
        return self.client.request_sensor_stream(message_cls)

    def init_episode(self, vehicle_configuration):
        self.send_vehicle_configuration(vehicle_configuration)
        self.request_map()

    def send_vehicle_configuration(self, vehicle_configuration):
        logging.getLogger("simulator").info('Sending vehicle configuration {0}'.format(vehicle_configuration.name))
        vehicle_response = self.request(messaging.JSONConfigurationCommand(
            vehicle_configuration.configuration, VEHICLE_CONFIG_COMMAND_UUID))
        
        if vehicle_response is None:
            logging.getLogger("network").error('Failed to send the vehicle configuration')
            raise ConnectionError('Failed to connect to the monodrive vehicle.')
        else:
            logging.getLogger("simulator").info('{0}'.format(vehicle_response))

    def send_simulator_configuration(self):
        logging.getLogger("simulator").info('Sending simulator configuration ip:{0}:{1}'.format(self.simulator_configuration.server_ip,self.simulator_configuration.server_port))
        simulator_response = self.request(messaging.JSONConfigurationCommand(
            self.simulator_configuration.configuration, SIMULATOR_CONFIG_COMMAND_UUID))
        if simulator_response is None:
            logging.getLogger("network").error('Failed to send the simulator configuration')
            raise ConnectionError('Failed to connect to the monodrive simulator.')
        else:
            logging.getLogger("simulator").info('{0}'.format(simulator_response))

    def send_scenario_init(self, scen):
        init = scen.init_scene_json
        msg = messaging.ScenarioInitCommand(init)
        return self.request(msg, timeout=180)

    def send_scenario(self, scen):
        json = scen.to_json
        msg = messaging.ScenarioModelCommand(scenario=json)
        return self.request(msg, timeout=180)

    def start_sensor_command(self, sensor_type, display_port, sensor_id, packet_size, drop_frames):
        """ Return server response from Sensor request. """
        u_sensor_type = u"{}".format(sensor_type)
        response = self.request_sensor_stream(
            messaging.StreamDataCommand(u_sensor_type, sensor_id, self.simulator_configuration.client_ip,
                                        display_port, u'tcp', 0, packet_size=packet_size, dropFrames=drop_frames))
        return response

    def stop_sensor_command(self, sensor_type, display_port, sensor_id, packet_size, drop_frames):
        """ Return server response from Sensor request. """
        u_sensor_type = u"{}".format(sensor_type)
        response = self.request(
            messaging.StreamDataCommand(u_sensor_type, sensor_id, self.simulator_configuration.client_ip,
                                    display_port, u'tcp', 1, packet_size=packet_size, dropFrames=drop_frames))

        return response

    def setup_logger(self):
        # Get the formatter to capitalize the logger name
        simple_formatter = MyFormatter("%(name)s-%(levelname)s: %(message)s")
        # detailed_formatter = MyFormatter("%(asctime)s %(name)s-%(levelname)s:[%(process)d]:  - %(message)s")

        for category, level in self.simulator_configuration.logger_settings.items():
            level = logging.getLevelName(level.upper())
            logger = logging.getLogger(category)
            logger.setLevel(level)

            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setLevel(level)
            console_handler.setFormatter(simple_formatter)

            file_handler = logging.handlers.RotatingFileHandler('client_logs.log', maxBytes=100000, backupCount=5)
            file_handler.setLevel(level)
            file_handler.setFormatter(simple_formatter)

            logger.addHandler(file_handler)
            logger.addHandler(console_handler)

    def request_map(self):
        command = messaging.MapCommand(self.simulator_configuration.map_settings)
        result = self.request(command, 60)
        self.map_data = result
        self.map = Map(result)
        self.map.start()


class MyFormatter(logging.Formatter):
    def format(self, record):
        record.name = record.name.upper()
        return logging.Formatter.format(self, record)




