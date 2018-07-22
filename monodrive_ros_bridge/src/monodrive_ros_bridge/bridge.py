#!/usr/bin/env python

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB), and the INTEL Visual Computing Lab.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.



"""
Rosbridge class:

Class that handle communication between mono and ROS
"""
import random, time
from itertools import count
from multiprocessing import Event

from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
import rospy


from monodrive import Simulator, SimulatorConfiguration, VehicleConfiguration
#from mono.settings import monoSettings
#from monodrive_ros_bridge.control import InputController
from monodrive_ros_bridge.markers import PlayerAgentHandler, NonPlayerAgentsHandler
from monodrive_ros_bridge.sensors import CameraHandler, GpsHandler, LidarHandler, ImuHandler

from monodrive.vehicles import SimpleVehicle


class RosVehicle(SimpleVehicle):
    def __init__(self, simulator_config, vehicle_config, restart_event=None, **kwargs):
        super(RosVehicle, self).__init__(simulator_config, vehicle_config, restart_event)
        self.running = False
        self.episode_event = Event()

    def start(self):
        rospy.loginfo("starting vehicle process")
        self.running = True
        super(RosVehicle, self).start()

    def run(self):
        rospy.loginfo("running vehicle process")
        while self.running:
            self.episode_event.wait()

    def stop(self):
        rospy.loginfo("stopping vehicle process")
        self.running = False
        super(RosVehicle, self).stop()

    def end_episode(self):
        self.episode_event.set()


class MonoRosBridge(object):
    """
    monoDrive Ros bridge
    """

    def __init__(self, params):
        """

        :param params: dict of parameters, see settings.yaml
        :param rate: rate to query data from mono in Hz
        """

        self.frames_per_episode = params['Framesperepisode']

        # Simulator configuration defines network addresses for connecting to the simulator and material properties
        simulator_config = SimulatorConfiguration(params['SimulatorConfig'])

        # Vehicle configuration defines ego vehicle configuration and the individual sensors configurations
        self.vehicle_config = VehicleConfiguration(params['VehicleConfig'])

        self.simulator = Simulator(simulator_config)
        self.vehicle = self.simulator.start_vehicle(self.vehicle_config, RosVehicle)
#        self.vehicle = RosVehicle(simulator_config, vehicle_config)

        self.param_sensors = params.get('sensors', {})

        self.tf_to_publish = []
        self.msgs_to_publish = []
        self.publishers = {}

        # definitions useful for time
        self.cur_time = rospy.Time.from_sec(
            0)  # at the beginning of simulation
        self.mono_game_stamp = 0
        self.mono_platform_stamp = 0

        # creating handler to handle vehicles messages
        self.player_handler = PlayerAgentHandler(
            "player_vehicle", process_msg_fun=self.process_msg)
        self.non_players_handler = NonPlayerAgentsHandler(
            "vehicles", process_msg_fun=self.process_msg)

        # creating handler for sensors
        self.sensors = {}
        print(self.param_sensors)
        for t, sensors in self.param_sensors.items():
            for id in sensors:
                self.add_sensor(sensors[id])

        # creating input controller listener
        #self.input_controller = InputController()

    def add_sensor(self, sensor):
        rospy.loginfo("Adding sensor {}".format(sensor))
        sensor_type = sensor['type']
        id = sensor['id'] #'{0}.{1}'.format(sensor_type, sensor['id'])
        sensor_handler = None
        if sensor_type == 'Lidar':
            sensor_handler = LidarHandler
        elif sensor_type == 'Camera':
            sensor_handler = CameraHandler
        elif sensor_type == 'IMU':
            sensor_handler = ImuHandler
        elif sensor_type == 'GPS':
            sensor_handler = GpsHandler

        if sensor_handler:
            if self.sensors.get(sensor_type, None) is None:
                self.sensors[sensor_type] = []

            self.sensors[sensor_type].append(sensor_handler(
                id,
                self.vehicle.get_sensor(sensor_type, id),
                process_msg_fun=self.process_msg))
        else:
            rospy.logerr(
                "Unable to handle sensor {name} of type {sensor_type}".format(
                    sensor_type=sensor_type, name=id))


    def on_shutdown(self):
        rospy.loginfo("Shutdown requested")

    def process_msg(self, topic=None, msg=None):
        """
        Function used to process message

        Here we create publisher if not yet created
        Store the message in a list (waiting for their publication) with their associated publisher

        Messages for /tf topics are handle differently in order to publish all transform in the same message
        :param topic: topic to publish the message on
        :param msg: monodrive_ros_bridge message
        """

        #rospy.loginfo("publishing on {0}".format(topic))

        if topic not in self.publishers:
            if topic == 'tf':
                self.publishers[topic] = rospy.Publisher(
                    topic, TFMessage, queue_size=100)
            else:
                self.publishers[topic] = rospy.Publisher(
                    topic, type(msg), queue_size=10)

        if topic == 'tf':
            # transform are merged in same message
            self.tf_to_publish.append(msg)
        else:
            self.msgs_to_publish.append((self.publishers[topic], msg))

    def send_msgs(self):
        for publisher, msg in self.msgs_to_publish:
            publisher.publish(msg)
        self.msgs_to_publish = []

        tf_msg = TFMessage(self.tf_to_publish)
        self.publishers['tf'].publish(tf_msg)
        self.tf_to_publish = []

    def compute_cur_time_msg(self):
        self.process_msg('clock', Clock(self.cur_time))

    def run(self):
        self.publishers['clock'] = rospy.Publisher(
            "clock", Clock, queue_size=10)

        rospy.loginfo('Starting Vehicle')
        # Start the Vehicle process
        #self.vehicle = self.simulator.start_vehicle(self.vehicle_config, RosVehicle)

        for frame in count():
            if (frame == self.frames_per_episode) or rospy.is_shutdown():
                rospy.loginfo("----- end episode -----")
                break

#            rospy.loginfo("waiting for data")
#            self.vehicle.sensor_data_ready.wait()

            rospy.loginfo("processing data")
            for sensor in self.vehicle.sensors:
                if self.sensors.get(sensor.type, None):
                    rospy.loginfo("getting data from {0}{1}".format(sensor.type,sensor.sensor_id))

                    processor = None
                    sensors = self.sensors[sensor.type]
                    for s in sensors:
                        if s.name == sensor.sensor_id:
                            processor = s

                    try:
                        data = sensor.q_vehicle.peek()
                        processor.process_sensor_data(data, self.cur_time)
                    except:
                        while not sensor.q_vehicle.empty():
                            data = sensor.q_vehicle.get()
                            processor.process_sensor_data(data, self.cur_time)

            rospy.loginfo("publishing messages")
            # publish all messages
            self.send_msgs()

            self.vehicle.sensor_data_ready.clear()
            control_data = self.vehicle.drive(self.vehicle.sensors, None)
            rospy.loginfo("--> {0}".format(control_data))
            self.vehicle.send_control_data(control_data)

            rospy.loginfo("waiting for data")
            self.vehicle.sensor_data_ready.wait()

            '''
            measurements, sensor_data = self.client.read_data()

            # handle time
            self.mono_game_stamp = measurements.game_timestamp
            self.cur_time = rospy.Time.from_sec(self.mono_game_stamp * 1e-3)
            self.compute_cur_time_msg()

            # handle agents
            self.player_handler.process_msg(
                measurements.player_measurements, cur_time=self.cur_time)
            self.non_players_handler.process_msg(
                measurements.non_player_agents, cur_time=self.cur_time)

            # handle sensors
            for name, data in sensor_data.items():
                self.sensors[name].process_sensor_data(data, self.cur_time)

            # publish all messages
            self.send_msgs()

            # handle control
            if rospy.get_param('mono_autopilot', True):
                control = measurements.player_measurements.autopilot_control
                self.client.send_control(control)
            else:
                control = self.input_controller.cur_control
                self.client.send_control(**control)
            '''

        # Waits for the restart event to be set in the control process
        self.vehicle.restart_event.wait()

        # Terminates control process
        self.vehicle.stop()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        rospy.loginfo("Exiting Bridge")
        return None
