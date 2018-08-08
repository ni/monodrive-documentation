__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import logging
import math
import time
import threading

from monodrive import SensorManager
from monodrive.networking import messaging

from monodrive.sensors import GPS, Waypoint


class BaseVehicle(object):
    def __init__(self, simulator, vehicle_config, restart_event=None, road_map = None, **kwargs):
        super(BaseVehicle, self).__init__()
        self.simulator = simulator
        self.simulator_configuration = simulator.simulator_configuration
        self.sensor_manager = SensorManager(vehicle_config, simulator)
        self.name = vehicle_config.id
        self.all_sensors_ready = self.sensor_manager.all_sensors_ready
        self.sensors = self.sensor_manager.sensor_list
        self.restart_event = restart_event
        self.last_time = 0.0
        self.update_sent = False
        self.scenario = None
        self.vehicle_state = None
        self.previous_control_sent_time = None
        self.control_thread = None
        self.b_control_thread_running = True
        self.road_map = road_map

    def start(self):
        self.sensor_manager.start()
        self.previous_control_sent_time = time.time()
        self.control_thread = threading.Thread(target=self.control_monitor)
        self.control_thread.start()

    def start_scenario(self, scenario):
        self.scenario = scenario
        self.vehicle_state = VehicleState()
        self.vehicle_state.start_scenario(scenario)
        self.start()

    def control_monitor(self):
        while self.b_control_thread_running:
            logging.getLogger("control").debug("Vehicle waiting on Sensor Data")
            #self.all_sensors_ready.wait()

            self.log_control_time(self.previous_control_sent_time)

            if self.vehicle_state is not None:
                self.vehicle_state.update_state(self.sensors)

            control_data = self.drive(self.sensors, self.vehicle_state)
            #self.all_sensors_ready.clear()
            self.send_control_data(control_data)

    def send_control_data(self, control_data):
        forward = control_data['forward']
        right = control_data['right']
        logging.getLogger("control").debug("Sending control data forward: %.4s, right: %.4s" % (forward, right))
        msg = messaging.EgoControlCommand(forward, right)
        resp = self.simulator.request(msg)
        if resp is None:
            logging.getLogger("control").error(
                "Failed response from sending control data forward: %s, right: %s" % (forward, right))
        self.previous_control_sent_time = time.time()
        for s in self.sensors:
            s.last_control_real_time.value = self.previous_control_sent_time

    def stop(self):
        # Stops all sensor streams and terminates processes
        self.b_control_thread_running = False
        self.sensor_manager.stop(self.simulator)
        logging.getLogger("vehicle").info("vehicle termitation complete")

    def drive(self, sensors, vehicle_state):
        raise NotImplementedError("To be implemented")

    def get_road_map(self):
        return self.road_map
        
    @staticmethod
    def log_control_time(previous_control_time):
        dif = time.time() - previous_control_time
        logging.getLogger("sensor").debug('Total Sensor Delay %.4f' % dif)

    @staticmethod
    def plan_target_lane(waypoint_sensor, vehicle_state=None):
        if vehicle_state is not None:
            return BaseVehicle.check_scenario_for_target_lane(vehicle_state)
        else:
            return waypoint_sensor.lane_number

    @staticmethod
    def check_scenario_for_target_lane(vehicle_state):
        if vehicle_state.target_lane_number is not None:
            return vehicle_state.target_lane_number
        else:
            return vehicle_state.estimated_lane_number

    def get_sensor(self, sensor_type, id):
        for sensor in self.sensors:
            if sensor.type == sensor_type and sensor.sensor_id == id:
                return sensor
        return None

    def get_sensors(self):
        return self.sensors


class VehicleState:
    def __init__(self):
        self.simulator_start_time = None
        self.starting_world_location = None
        self.simulation_time = None
        self.gps_location = None
        self.world_location = None
        self.sensors = None
        self.speed = None
        self.simulator_lane_number = None
        self.estimated_lane_number = None
        self.target_lane_number = None
        self.scenario = None
        self.story = None
        self.sequence = None
        self.maneuver_execution_count = None
        self.maneuver_current_event_index = None
        self.event = None
        self.complete_events = []

    def update_state(self, sensors):
        if self.simulator_start_time is None:
            self.set_initial_state(sensors)

        self.sensors = sensors
        self.update_gps_state()
        self.update_waypoint_state()
        self.check_scenario()

    def update_gps_state(self):
        gps_sensor = GPS.get_sensor(self.sensors)
        gps_sensor.get_message()
        self.simulation_time = gps_sensor.game_time
        self.gps_location = [gps_sensor.lat, gps_sensor.lng]
        self.world_location = gps_sensor.world_location
        self.speed = gps_sensor.speed

    def update_waypoint_state(self):
        waypoint_sensor = Waypoint.get_sensor(self.sensors)
        waypoint_sensor.get_message()
        self.simulator_lane_number = waypoint_sensor.lane_number
        self.estimated_lane_number = waypoint_sensor.get_estimated_current_lane(self.world_location)

    def set_initial_state(self, sensors):
        gps_sensor = GPS.get_sensor(sensors)
        gps_sensor.get_message()
        # Needed to calculate total time elapsed for condition check
        self.simulator_start_time = gps_sensor.game_time
        # Needed to calculate total distance traveled for condition check
        self.starting_world_location = gps_sensor.world_location

    def start_scenario(self, scenario):
        # Runs check_scenario after initial read of sensor values to have vehicle state
        self.scenario = scenario
        # Example scenario only have one story
        self.story = self.scenario.storyboard.stories[0]

    def setup_act(self, act):
        # Acts have one sequence
        self.sequence = act.sequence
        self.maneuver_execution_count = self.sequence.executions
        # Maneuvers have events that need to fire in order
        self.maneuver_current_event_index = 0

    @staticmethod
    def check_conditions(conditions, vehicle_state):
        """ Returns true if all the conditions are met, false if not"""
        return all(con.is_condition_ready(vehicle_state) for con in conditions)

    def check_scenario(self):
        if self.sequence and self.maneuver_execution_count > 0 and self.event is None:
            # Sequence devices number of executions, actors involved and the Maneuver
            next_possible_event = self.sequence.maneuver.events[self.maneuver_current_event_index]
            if self.check_conditions(next_possible_event.start_conditions, self):
                logging.getLogger("scenario").info('Setting Event', next_possible_event.name)
                self.event = next_possible_event
                self.event.start(self)
        elif self.sequence is None:
            # Get next act based on current vehicle state
            if self.check_conditions(self.story.act.start_conditions, self) is not None:
                logging.getLogger("scenario").info('Setting Act', self.story.act.name)
                self.setup_act(self.story.act)

        if self.event:
            self.event.is_event_complete(self)

    def event_complete(self):
        logging.getLogger("scenario").info('Completed Event', self.event.name)

        self.event.finish(self)

        # If all events in maneuver have been fired, else move on to next event in maneuver
        if len(self.sequence.maneuver.events) - 1 == self.maneuver_current_event_index:
            self.maneuver_execution_count -= 1
            map(lambda e: e.reset(), self.complete_events)
            self.complete_events = []

        else:
            self.maneuver_current_event_index += 1
            self.complete_events.append(self.event)

        self.event = None

        if self.maneuver_execution_count == 0:
            logging.getLogger("scenario").info('Scenario Complete!')

    @property
    def simulation_elapsed_time(self):
        return self.simulator_start_time - self.simulation_time

    @property
    def total_distance_traveled(self):
        dif = self.world_location - self.starting_world_location
        return math.sqrt(dif[0] ** 2 + dif[1] ** 2)


from .md_vehicle import MDVehicle
from .simple_vehicle import SimpleVehicle
from .teleport_vehicle import TeleportVehicle
