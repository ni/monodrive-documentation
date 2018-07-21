from multiprocessing import Event

from monodrive.networking import messaging
from monodrive.networking.client import Client
from monodrive.constants import *

from monodrive import VehicleConfiguration


class Simulator:

    def __init__(self, simulator_configuration):
        self.simulator_configuration = simulator_configuration
        self.restart_event = Event()
        self.vehicle_process = None
        self.scenario = None
        self._client = None

    def start_scenario(self, scenario, vehicle_class):

        self.scenario = scenario

        # Send both simulator configuration and scenario
        # self.send_simulator_configuration(self.simulator_configuration)
        self.send_scenario(scenario)
        print('Received Response From Sending Scenario')

        # Get Ego vehicle configuration from scenario, use that to create vehicle process
        vehicle_configuration = scenario.ego_vehicle_config
        vehicle_configuration = VehicleConfiguration.init_from_json(vehicle_configuration.to_json)
        self.vehicle_process = vehicle_class(self.simulator_configuration, vehicle_configuration, self.restart_event)

        # Start the Vehicle process
        self.vehicle_process.start_scenario(scenario)

    def start_vehicle(self, vehicle_configuration, vehicle_class):

        self.send_simulator_configuration(self.simulator_configuration)
        self.send_vehicle_configuration(vehicle_configuration)

        # Create vehicle process form received class
        self.vehicle_process = vehicle_class(self.simulator_configuration, vehicle_configuration, self.restart_event)

        # Start the Vehicle process
        self.vehicle_process.start()
        return self.vehicle_process

    def stop(self):
        self.vehicle_process.stop()

    @property
    def client(self):
        if self._client is None:
            self._client = Client((self.simulator_configuration.server_ip,
                                   self.simulator_configuration.server_port))

        if not self._client.isconnected():
            self._client.connect()
        return self._client

    def request(self, message_cls, timeout=90):
        return self.client.request(message_cls, timeout)

    def request_sensor_stream(self, message_cls):
        return self.client.request_sensor_stream(message_cls)

    def send_vehicle_configuration(self, vehicle_configuration):
        vehicle_response = self.request(messaging.JSONConfigurationCommand(
            vehicle_configuration.configuration, VEHICLE_CONFIG_COMMAND_UUID))
        if vehicle_response is None:
            raise ConnectionError('Failed to connect to the monodrive vehicle.')

    def send_simulator_configuration(self, simulator_configuration):
        simulator_response = self.request(messaging.JSONConfigurationCommand(
            simulator_configuration.configuration, SIMULATOR_CONFIG_COMMAND_UUID))
        if simulator_response is None:
            raise ConnectionError('Failed to connect to the monodrive simulator.')

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
        #response = self.request_sensor_stream(
        #    messaging.StreamDataCommand(u_sensor_type, sensor_id, self.simulator_configuration_manager.client_ip,
        #                                display_port, u'udp', 0, packet_size=packet_size, dropFrames=drop_frames))
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

