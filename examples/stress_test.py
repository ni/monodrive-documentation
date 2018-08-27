#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import argparse
import json
import math
import random
import threading
import time

from monodrive import Simulator
from monodrive.configuration import SimulatorConfiguration, VehicleConfiguration
from monodrive.constants import SIMULATOR_STATUS_UUID, ClockMode_Continuous, ClockMode_AutoStep, ClockMode_ClientStep
from monodrive.networking import messaging
from monodrive.sensors import GPS


parser = argparse.ArgumentParser(description='monoDrive simulator stress test script')
parser.add_argument('--sim-config', default='simulator.json')
parser.add_argument('--vehicle-config', default='test.json')
#parser.add_argument('--clock-mode', default=None, help='override clock mode',
#                    choices=['Continuous', 'AutoStep', 'ClientStep'])
#parser.add_argument('--fps', help='override sensor fps', type=int)
parser.add_argument('--exclude',
                    help='comma separated list of sensors to exclude (sensors not in list will be included)')
parser.add_argument('--include',
                    help='comma separated list of sensors to include (sensors not in list will be excluded)')


millis = lambda: int(round(time.time() * 1000))

class Tracker:
    def __init__(self):
        self.last_location = None
        self.lock = threading.Lock()
        self.last_distance = 0

    def update(self, location):
        self.lock.acquire()
        if self.last_location is not None:
            self.last_distance = self.haversine(location, self.last_location)
        self.last_location = location
        self.lock.release()

    def get_last_distance_travelled(self):
        self.lock.acquire()
        location = self.last_location if self.last_location is not None else {}
        distance = self.last_distance
        self.lock.release()
        return location, distance

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
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        self.running = True
        packets = 0
        start = None
        timer = millis()
        get_time = lambda: data.get('game_time', millis()) if isinstance(data, dict) and self.clock_mode is not 0 else millis()
        time_units = lambda: 'real time' if (isinstance(data, dict) and data.get('game_time', None) is None) or self.clock_mode is 0 else 'game time'
        while self.running:
            data = self.sensor.get_message(timeout=.5)
            if data is None:
                continue

            if data:
                packets += 1

                if isinstance(self.sensor, GPS):
                    self.tracker.update(data)
            else:
                continue

            if start is None:
                start = get_time()
                print("start: %f" % start)

            if millis() - timer >= 1000:
                seconds = (get_time() - start) / 1000
                # print("game time: %f, diff: %f" % (get_time(), seconds))
                #if (data.get('game_time', None) is None):
                #    seconds /= 1000

                try:
                    fps = packets / seconds
                except:
                    fps = 0

                location, distance = self.tracker.get_last_distance_travelled()
                print('{0}: {1:.2f} fps ({2} frames received in {3:.2f} secs ({4})), distance: {5:.4f}, speed: {6:.4f}'.format(
                    self.sensor.name, fps, packets, seconds, time_units(), distance, location.get('speed',0)))
                packets = 0
                timer = millis()
                start = get_time()

        print("  THREAD COMPLETED %s" % self.sensor.name)

    def stop(self):
        print("stopping %s" % self.sensor.name)
        self.sensor.stop()
        self.running = False

    def join(self):
        if self.thread is not None:
            self.thread.join()

def run_test(simulator, vehicle_config, clock_mode, fps):
    print("======  test start  ======")
    print("  running test. clock-mode: {0}, fps: {1}".format(clock_mode, fps))
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

    print("  starting %d sensors" % len(sensors))
    tasklist = []
    tracker = Tracker()
    for sensor in sensors:
        print("--> %s" % sensor.name)
        sensor.start()
        print(sensor.send_start_stream_command(simulator))

    print("starting sensor tasks")
    for sensor in sensors:
        try:
            st = SensorTask(sensor, clock_mode, tracker)
            tasklist.append(st)
        except Exception as e:
            print(str(e))

    print("  waiting on sensors")
    for sensor in sensors:
        print(" waiting on %s" % sensor.name)
        sensor.socket_ready_event.wait()
        print(" %s ready" % sensor.name)

    print("  sampling sensors")
    #msg = messaging.EgoControlCommand(random.randrange(-5.0, 5.0), random.randrange(-3.0, 3.0)) # drive randomly
    msg = messaging.EgoControlCommand(2.5, 0.0) # go straight
    for _ in range(0, 100):
        simulator.request(msg)
        time.sleep(.2)
#        print(simulator.request(messaging.Message(SIMULATOR_STATUS_UUID)))

    print("  stopping")
    for task in tasklist:
        print(task.sensor.send_stop_stream_command(simulator))
        task.stop()

    print("  waiting on sensors to exit")
    for sensor in sensors:
        sensor.join()

    for task in tasklist:
        task.join()

    print("======  test end  ======\n\n")


if __name__ == "__main__":
    args = parser.parse_args()

    sim_config = SimulatorConfiguration(args.sim_config)
    vehicle_config = VehicleConfiguration(args.vehicle_config)

    sim_config.client_settings['logger']['sensor']='debug'
    sim_config.client_settings['logger']['network']='debug'

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

    # if args.fps:
    #     for sensor in vehicle_config.sensor_configuration:
    #         sensor['fps'] = args.fps

    print(json.dumps(vehicle_config.configuration))
    simulator = Simulator(sim_config)

    for fps in range(30, 50, 10):
        modes = [ClockMode_Continuous, ClockMode_AutoStep, ClockMode_ClientStep]
        random.shuffle(modes)
        for clock_mode in modes:
            run_test(simulator, vehicle_config, clock_mode, fps)
            time.sleep(1)


