import logging
import math
import time
import threading
import cPickle as pickle

import sys, traceback

from monodrive.networking import messaging
from monodrive.sensors import GPS, Waypoint
from monodrive.networking.queues import SingleQueue

import multiprocessing


class BaseVehicle(object):
    def __init__(self, simulator, vehicle_config, restart_event=None, road_map = None, **kwargs):
        super(BaseVehicle, self).__init__()
        self.simulator = simulator
        self.simulator_config = simulator.simulator_configuration
        self.name = vehicle_config.id
        self.sensors = []
        self.restart_event = restart_event
        self.last_time = 0.0
        self.update_sent = False
        self.scenario = None
        self.vehicle_state = None
        self.previous_control_sent_time = None
        self.control_thread = None
        self.b_control_thread_running = True
        self.road_map = road_map
        self.q_road_map = SingleQueue()
        self.q_road_map.put(self.road_map)

        #FROM old sensor manager
        self.vehicle_config = vehicle_config
        self.simulator = simulator
        self.sensor_process_dict = {}
        self.initialized = self.init_sensors()
        self.vehicle_update_rate = .1 # ticks per second
        self.vehicle_running = True

    def init_vehicle_loop(self):
        self.vehicle_thread = threading.Thread(target=self.vehicle_loop)
        self.vehicle_thread.start()
    
    def vehicle_loop(self):
        #step the vehicle to start the measurements
        self.step({'forward':0.0,'right':0.0})
        sensors = self.get_sensors()
        while self.vehicle_running:
            time.sleep(self.vehicle_update_rate)
            control = self.drive(sensors)
            self.step(control)
            

    def step(self, control_data):
        self.control_thread = threading.Thread(target=self.do_control_thread(control_data))
        self.control_thread.start()

    def do_control_thread(self, control_data):
        forward = control_data['forward']
        right = control_data['right']
        logging.getLogger("control").debug("Sending control data forward: %.4s, right: %.4s" % (forward, right))
        msg = messaging.EgoControlCommand(forward, right)
        resp = self.simulator.request(msg)
        if resp is None:
            logging.getLogger("control").error(
                "Failed response from sending control data forward: %s, right: %s" % (forward, right))
        #self.previous_control_sent_time = time.time()
        #for s in self.sensors:
            #s.last_control_real_time.value = self.previous_control_sent_time

    def drive(self, sensors):
        raise NotImplementedError("To be implemented in base class")

    def get_road_map(self):
        msg = self.q_road_map.peek()
        return msg

    def get_sensor(self, sensor_type, id):
        for sensor in self.sensors:
            if sensor.type == sensor_type and sensor.sensor_id == id:
                return sensor
        return None

    def get_sensors(self):
        return self.sensors

    def init_sensors(self):
        for sensor_config in self.vehicle_config.sensor_configuration:
            if not sensor_config['sensor_process']:
                continue
            sensor_instance = self.init_sensor(sensor_config)
            self.sensor_process_dict[sensor_instance.name] = sensor_instance
            self.sensors.append(sensor_instance) 

    def init_sensor(self, sensor_config):
        sensor_type = sensor_config['type']
        _Sensor_Class = self.vehicle_config.get_class(sensor_type)
        sensor_instance = _Sensor_Class(sensor_type, sensor_config, self.simulator_config)
        return sensor_instance



    def get_process_list(self):
        _processes = []
        for sensor in self.sensor_process_dict.values():
            _processes.append(sensor)
            
            if getattr(sensor, 'packetizer_process', None) is not None:
                _processes.append(sensor.packetizer_process)
        return _processes

    def start(self):
        print("start processes")
        [p.start() for p in self.get_process_list()]
        
        for s in self.sensors:
            s.sensor_initialized_event.wait()
            print("{0} processes started".format(s.name))

        for s in self.sensors:
            res = s.send_start_stream_command(self.simulator)
            if not res.is_success:
                logging.getLogger("sensor").error(
                    "Failed start stream command for sensor %s %s" % (s.name, res.error_message))
            else:
                logging.getLogger("sensor").info("Sensor ready %s" % s.name)

        for s in self.sensors:
            s.socket_ready_event.wait()


        logging.getLogger("vehicle").info("starting vehicle loop")
        # Kicks off simulator for stepping
        self.init_vehicle_loop()

    def stop(self, simulator):
        #stop monitoring sensors
        

        self.b_sensor_monitor_thread_running = False 

        #stopping simulator from sending data
        logging.getLogger("sensor").info("sensor manager stopping sensor streaming")
        [s.send_stop_stream_command(simulator) for s in self.sensors]
        

        logging.getLogger("sensor").info("sensor manager stopping sensor listening")
        [s.stop() for s in self.sensors]
        logging.getLogger("sensor").info("sensor process TERMINATING")
        [s.terminate() for s in self.sensors]
        logging.getLogger("sensor").info("joining sensor processes STARTED")
        self.print_all_stacktraces()
        [s.join() for s in self.sensors]
        logging.getLogger("sensor").info("joining sensor processes COMPLETE")

        logging.getLogger("sensor").info("sensor termitation complete")

    def print_all_stacktraces(self):
        print("\n*** STACKTRACE - START ***\n")
        code = []
        for threadId, stack in sys._current_frames().items():
            threadName = ''
            for t in threading.enumerate():
                if t.ident == threadId:
                    threadName = t.name
            code.append("\n# ThreadID: %s %s" % (threadId, threadName))
            for filename, lineno, name, line in traceback.extract_stack(stack):
                code.append('File: "%s", line %d, in %s' % (filename,
                                                            lineno, name))
                if line:
                    code.append("  %s" % (line.strip()))
        for line in code:
            print(line)
        print("\n*** STACKTRACE - END ***\n")


