#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import argparse
import logging
import math
import signal
import sys
import threading
import time

from monodrive import Simulator
from monodrive.configuration import SimulatorConfiguration, VehicleConfiguration
from monodrive.constants import ClockMode_Continuous, ClockMode_AutoStep, ClockMode_ClientStep
from monodrive.networking import messaging
from monodrive.sensors import GPS


LOG_CATEGORY = "test"

parser = argparse.ArgumentParser(description='monoDrive simulator stress test script')
parser.add_argument('--sim-config', default='simulator.json')
parser.add_argument('--vehicle-config', default='test.json')
parser.add_argument('--clock-mode', default=ClockMode_Continuous, help='specify clock mode, default is Continuous',
                    choices=['Continuous', 'AutoStep', 'ClientStep'])
parser.add_argument('--exclude',
                    help='comma separated list of sensors to exclude (sensors not in list will be included)')
parser.add_argument('--include',
                    help='comma separated list of sensors to include (sensors not in list will be excluded)')


millis = lambda: int(round(time.time() * 1000))

class Tracker:
    def __init__(self):
        self.last_location = None
        self.lock = threading.Lock()
        self.start_location = None
        self.last_distance = 0
        self.speed = 0

    def update(self, location):
        self.lock.acquire()

        if self.start_location is None:
            self.start_location = location

        if self.last_location is not None:
            self.last_distance = self.haversine(location, self.last_location)
        self.last_location = location

        self.speed += location.get('speed',0)

        self.lock.release()

    def get_last_distance_travelled(self):
        self.lock.acquire()
        location = self.last_location if self.last_location is not None else {}
        distance = self.last_distance
        self.lock.release()
        return location, distance

    def total_distance(self):
        return self.haversine(self.start_location, self.last_location)

    def haversine(self, location1, location2):
        lat1, lng1, lat2, lng2 = map(math.radians, [location1['lat'], location1['lng'], location2['lat'], location2['lng']])
        dlat = lat2 - lat1
        dlng = lng2 - lng1
        a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlng / 2)**2
        c = 2 * math.asin(math.sqrt(a))
        R = 6371000
        return c * R


class SensorTask:
    def __init__(self, sensor, clock_mode, tracker):
        self.sensor = sensor
        self.clock_mode = clock_mode
        self.tracker = tracker
        self.data_received = threading.Event()
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        self.running = True
        packets = 0
        start = None

        get_time = lambda: data.get('game_time', millis()) \
            if isinstance(data, dict) and self.clock_mode is not 0 \
            else millis()

        time_units = lambda: 'real time' \
            if (isinstance(data, dict) and data.get('game_time', None) is None) or self.clock_mode is 0 \
            else 'game time'

        while self.running and self.sensor.is_sensor_running():
            data = self.sensor.get_message(timeout=.5)
            if data is None:
                continue

            if start is None:
                start = get_time()

            packets += 1

            if isinstance(self.sensor, GPS):
                self.tracker.update(data)

            self.data_received.set()

        try:
            seconds = (get_time() - start) / 1000
            fps = packets / seconds

            logging.getLogger(LOG_CATEGORY).info(
                '{0: >18}: {1:5.2f} fps ({2:3} frames received in {3:6.2f} secs ({4})), distance: {5:6.4f}, avg speed: {6:6.4f}'.format(
                    self.sensor.name, fps, packets, seconds, time_units(), self.tracker.total_distance(),
                    self.tracker.speed / packets))
        except:
            pass

    def stop(self):
        #logging.getLogger(LOG_CATEGORY).info("stopping %s" % self.sensor.name)
        self.sensor.stop()
        self.running = False

    def join(self):
        if self.thread is not None:
            self.thread.join()


def run_test(simulator, vehicle_config, clock_mode, fps):
    logging.getLogger(LOG_CATEGORY).info("======  test start  ======")
    logging.getLogger(LOG_CATEGORY).info("  running test. clock-mode: {0}, fps: {1}".format(clock_mode, fps))
    vehicle_config.configuration['clock_mode'] = clock_mode
    for sensor in vehicle_config.sensor_configuration:
        sensor['fps'] = fps

    simulator.send_configuration()
    simulator.send_vehicle_configuration(vehicle_config)

    sensors = []
    idx = 0
    for sensor_config in vehicle_config.sensor_configuration:
        if sensor_config['sensor_process']:
            sensor_class = vehicle_config.get_class(sensor_config['type'])
            sensors.append(sensor_class(idx, sensor_config, sim_config))
            idx = idx + 1

    logging.getLogger(LOG_CATEGORY).info("  starting %d sensors" % len(sensors))
    tasklist = []
    tracker = Tracker()
    for sensor in sensors:
        sensor.start()
        sensor.send_start_stream_command(simulator)

    for sensor in sensors:
        try:
            st = SensorTask(sensor, clock_mode, tracker)
            tasklist.append(st)
        except Exception as e:
            print(str(e))

    for sensor in sensors:
        sensor.socket_ready_event.wait()

    #msg = messaging.EgoControlCommand(random.randrange(-5.0, 5.0), random.randrange(-3.0, 3.0)) # drive randomly
    accel = 0.5
    accel_total = 0
    msg = messaging.EgoControlCommand(accel, 0.0) # go straight

    for _ in range(0, 60):
        accel_total += accel
        simulator.request(msg)
        if clock_mode == ClockMode_ClientStep:
            for st in tasklist:
                st.data_received.wait()
                st.data_received.clear()
        else:
            time.sleep(.2)

    simulator.request(messaging.EgoControlCommand(-accel_total, 0.0))
    simulator.request(messaging.EgoControlCommand(0.0, 0.0))

    for task in tasklist:
        task.sensor.send_stop_stream_command(simulator)
        task.stop()

    for sensor in sensors:
        sensor.join()

    for task in tasklist:
        task.join()

    logging.getLogger(LOG_CATEGORY).info("======  test end  ======\n\n")


def shutdown(sig, frame):
    #print('shutting down...')
    time.sleep(5)
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, shutdown)
    # set up logging
    logging.basicConfig(level=logging.DEBUG, format="%(name)-12s %(levelname)-8s: %(message)s")

    args = parser.parse_args()

    sim_config = SimulatorConfiguration(args.sim_config)
    vehicle_config = VehicleConfiguration(args.vehicle_config)

    sim_config.client_settings['logger']['sensor']='debug'
    sim_config.client_settings['logger']['network']='debug'
    simulator = Simulator(sim_config)

    if args.include:
        sensor_ids = args.include.split(',')
        sensors = vehicle_config.sensor_configuration

        list = []
        for sensor in sensors:
            if sensor['type'] in sensor_ids or sensor['type']+':'+sensor['id'] in sensor_ids:
                sensor['sensor_process'] = True
                list.append(sensor)

        vehicle_config.sensor_configuration = list
        vehicle_config.configuration['sensors'] = list

    elif args.exclude:
        sensor_ids = args.exclude.split(',')
        sensors = vehicle_config.sensor_configuration

        list = []
        for sensor in sensors:
            if sensor['type'] in sensor_ids or sensor['type'] + ':' + sensor['id'] in sensor_ids:
                continue

            list.append(sensor)

        vehicle_config.sensor_configuration = list
        vehicle_config.configuration['sensors'] = list


    #logging.getLogger(LOG_CATEGORY).debug(json.dumps(vehicle_config.configuration))

    clock_mode = ClockMode_Continuous
    if args.clock_mode == 'AutoStep':
        clock_mode = ClockMode_AutoStep
    elif args.clock_mode == 'ClientStep':
        clock_mode = ClockMode_ClientStep

    # run test
    for fps in range(10, 110, 10):
        run_test(simulator, vehicle_config, clock_mode, fps)
        time.sleep(1)

    # reset vehicle and stop
    simulator.send_vehicle_configuration(vehicle_config)
    simulator.stop()



