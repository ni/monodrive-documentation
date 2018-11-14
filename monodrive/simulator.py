
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import logging
from logging.handlers import RotatingFileHandler
from multiprocessing import Event
import sys
try:
    import psutil
except:
    pass

from monodrive.networking import messaging
from monodrive.networking.client import Client
from monodrive.constants import *

from monodrive import VehicleConfiguration


class Simulator(object):

    def __init__(self, configuration):
        self.configuration = configuration
        self.restart_event = Event()
        self.ego_vehicle = None
        self.scenario = None
        self._client = None
        self.setup_logger()
        self.map_data = None

    def start_scenario(self, scenario, vehicle_class):
        self.scenario = scenario

        # Send both simulator configuration and scenario
        # self.send_configuration(self.configuration)
        self.send_scenario(scenario)

        # Get Ego vehicle configuration from scenario, use that to create vehicle process
        vehicle_configuration = scenario.ego_vehicle_config
        vehicle_configuration = VehicleConfiguration.init_from_json(vehicle_configuration.to_json)
        self.ego_vehicle = vehicle_class(self, vehicle_configuration, self.restart_event)

        # Start the Vehicle process
        self.ego_vehicle.start_scenario(scenario)


    def get_ego_vehicle(self, vehicle_configuration, vehicle_class):
        # Create vehicle process form received class
        self.map_data = self.request_map()
        if not self.map_data:
            logging.getLogger("simulator").error("failed to get map")
            return None

        self.ego_vehicle = vehicle_class(self, vehicle_configuration, self.restart_event, self.map_data)
        return self.ego_vehicle

    def stop(self):

        # Stop all processes
        logging.getLogger("simulator").info("start shutting down simulator client")
        if self.ego_vehicle is not None:
            self.ego_vehicle.stop()
        logging.getLogger("simulator").info("simulator client shutdown complete")


        # Disconnect from server
        self.client.disconnect()
        self.client.stop()

        ## get the pid of this program
        #pid=os.getpid()

        ## when you want to kill everything, including this program
        #self.kill_process_tree(pid, False)

    def kill_process_tree(self, pid, including_parent=True):
        parent = psutil.Process(pid)
        for child in parent.children(recursive=True):
            child.kill()

        if including_parent:
            parent.kill()

    @property
    def client(self):
        if self._client is None:
            self._client = Client((self.configuration.server_ip,
                                   self.configuration.server_port))

        if not self._client.isconnected():
            self._client.connect()
        return self._client

    def request(self, message_cls, timeout=5):
        return self.client.request(message_cls, timeout)

    def request_sensor_stream(self, message_cls):
        # wait for 2 responses when requesting the sensor to stream data
        # the second will include a sensor_ready flag
        messages = self.client.request(message_cls, 10, 2)
        return messages[1]

    def send_vehicle_configuration(self, vehicle_configuration):
        logging.getLogger("simulator").info('Sending vehicle configuration {0}'.format(vehicle_configuration.name))
        vehicle_response = self.request(messaging.JSONConfigurationCommand(
            vehicle_configuration.configuration, VEHICLE_CONFIG_COMMAND_UUID))
        
        if vehicle_response is None:
            logging.getLogger("network").error('Failed to send the vehicle configuration')

        #else:
        #    logging.getLogger("simulator").debug('{0}'.format(vehicle_response))
        return vehicle_response

    def send_configuration(self):
        logging.getLogger("simulator").info('Sending simulator configuration ip:{0}:{1}'.format(self.configuration.server_ip,self.configuration.server_port))
        simulator_response = self.request(messaging.JSONConfigurationCommand(
            self.configuration.configuration, SIMULATOR_CONFIG_COMMAND_UUID))
        if simulator_response is None:
            logging.getLogger("network").error('Failed to send the simulator configuration')

        #else:
        #    logging.getLogger("simulator").debug('{0}'.format(simulator_response))

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
            messaging.StreamDataCommand(u_sensor_type, sensor_id, self.configuration.client_ip,
                                        display_port, u'tcp', 0, packet_size=packet_size, dropFrames=drop_frames))
        return response

    def stop_sensor_command(self, sensor_type, display_port, sensor_id, packet_size, drop_frames):
        """ Return server response from Sensor request. """
        u_sensor_type = u"{}".format(sensor_type)
        response = self.request(
            messaging.StreamDataCommand(u_sensor_type, sensor_id, self.configuration.client_ip,
                                    display_port, u'tcp', 1, packet_size=packet_size, dropFrames=drop_frames),
            timeout=2)

        return response

    def setup_logger(self):
        simple_formatter = LevelNameFormatter("%(levelname)-8s:%(name)-10s: %(message)s")
        color_formatter = ColorFormatter("%(levelname)-15s:%(name)-10s: %(message)s")

        stream_handler = logging.StreamHandler(sys.stdout)
        stream_handler.setFormatter(color_formatter)

        # add default handler
        level = logging.getLevelName("DEBUG")
        file_handler = logging.handlers.RotatingFileHandler('client_logs.log', mode="w")
        file_handler.setLevel(level)
        file_handler.setFormatter(simple_formatter)

        logging.root.setLevel(level)
        logging.root.addHandler(file_handler)
        logging.root.addHandler(stream_handler)

        # add category handlers
        for category, level in self.configuration.logger_settings.items():
            levelname = level.upper()
            level = logging.getLevelName(levelname)
            logger = logging.getLogger(category)
            logger.setLevel(level)

            file_handler = logging.handlers.RotatingFileHandler('client_logs.log', mode="w")
            file_handler.setLevel(level)
            file_handler.setFormatter(simple_formatter)

            logger.addHandler(file_handler)
            logger.addHandler(stream_handler)

    def request_map(self):
        command = messaging.MapCommand(self.configuration.map_settings)
        result = self.request(command, 60)
        if result is not None and result.data:
            self.map_data = result.data
            return result.data
        else:
            return None

    def spawn_object(self, spawn_type, location, rotation):
        command = messaging.SpawnCommand({"type": spawn_type, "location": location, "rotation": rotation})
        result = self.request(command)
        if result.data:
            return result.data["id"]
        else:
            return None

    def move_actor(self, id, location, rotation):
        command = messaging.MoveActorCommand({"id": id, "location": location, "rotation": rotation})
        result = self.request(command)
        if result.data:
            return result.data["success"]
        else:
            return None
        

class LevelNameFormatter(logging.Formatter):
    def format(self, record):
        record.name = record.name.upper()
        return logging.Formatter.format(self, record)


class ColorFormatter(logging.Formatter):
    WHITE, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, GRAY, LIGHTER_GRAY, LIGHT_GRAY = range(10)

    COLORS = {
        'WARNING': YELLOW,
        'INFO': LIGHT_GRAY,
        'DEBUG': BLUE,
        'CRITICAL': YELLOW,
        'ERROR': RED
    }

    # These are the sequences need to get colored ouput
    RESET_SEQ = "\033[0m"
    COLOR_SEQ = "\033[1;%dm"
    BOLD_SEQ = "\033[1m"

    def format(self, record):
        record.name = record.name.upper()
        levelname = record.levelname
        if levelname in ColorFormatter.COLORS:
            record.levelname = ColorFormatter.COLOR_SEQ % (30 + ColorFormatter.COLORS[levelname]) + levelname
        return logging.Formatter.format(self, record)




