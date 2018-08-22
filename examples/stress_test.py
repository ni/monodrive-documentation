#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import argparse
import json
import threading
import time

from monodrive import Simulator
from monodrive.configuration import SimulatorConfiguration, VehicleConfiguration
from monodrive.constants import SIMULATOR_STATUS_UUID
from monodrive.networking import messaging


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

class SensorTask:
    def __init__(self, sensor, clock_mode):
        self.sensor = sensor
        self.clock_mode = clock_mode

    def run(self):
        self.running = True
        packets = 0
        start = None
        timer = millis()
        get_time = lambda: data.get('game_time', millis()) if isinstance(data, dict) and self.clock_mode is not 0 else millis()
        time_units = lambda: 'real time' if (isinstance(data, dict) and data.get('game_time', None) is None) or self.clock_mode is 0 else 'game time'
        while self.running:
            data = self.sensor.get_message()
            if data:
                packets += 1
            else:
                continue

            if start is None:
                start = get_time()
                print("start: %f" % start)

            if millis() - timer >= 5000:
                seconds = (get_time() - start) / 1000
                # print("game time: %f, diff: %f" % (get_time(), seconds))
                #if (data.get('game_time', None) is None):
                #    seconds /= 1000

                try:
                    fps = packets / seconds
                except:
                    fps = 0
                print('{0}: {1:.2f} fps ({2} frames received in {3:.2f} secs ({4}))'.format(
                    self.sensor.name, fps, packets, seconds, time_units()))
                packets = 0
                timer = millis()
                start = get_time()

    def stop(self):
        self.sensor.stop()
        self.running = False


def run_test(simulator, vehicle_config, clock_mode, fps):
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

    print("starting sensors")
    tasklist = []
    for sensor in sensors:
        sensor.start()
        sensor.send_start_stream_command(simulator)

        st = SensorTask(sensor, clock_mode)
        tasklist.append(st)
        t = threading.Thread(target=st.run)
        t.start()

    print("waiting on sensors")
    for sensor in sensors:
        sensor.socket_ready_event.wait()

    print("sampling sensors")
    if clock_mode == 2:
        msg = messaging.EgoControlCommand(0.0, 0.0)
        for _ in range(0, 1000):
            simulator.request(msg)
            #time.sleep(.1)
    else:
        for _ in range(0, 2):
            time.sleep(30)
            simulator.request(messaging.Message(SIMULATOR_STATUS_UUID))

    print("stopping")
    for task in tasklist:
        task.stop()

    print("waiting on sensors to exit")
    for sensor in sensors:
        sensor.send_stop_stream_command(simulator)
        sensor.join()


if __name__ == "__main__":
    args = parser.parse_args()

    sim_config = SimulatorConfiguration(args.sim_config)
    vehicle_config = VehicleConfiguration(args.vehicle_config)

    sim_config.client_settings['logger']['sensor']='debug'
    sim_config.client_settings['logger']['network']='debug'

    # if args.clock_mode:
    #     if args.clock_mode == 'Continuous':
    #         vehicle_config.configuration['clock_mode'] = 0
    #     elif args.clock_mode == 'AutoStep':
    #         vehicle_config.configuration['clock_mode'] = 1
    #     elif args.clock_mode == 'ClientStep':
    #         vehicle_config.configuration['clock_mode'] = 2

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
        for clock_mode in range(0, 3):
            print("running test. clock-mode: {0}, fps: {1}".format(clock_mode, fps))
            run_test(simulator, vehicle_config, clock_mode, fps)


