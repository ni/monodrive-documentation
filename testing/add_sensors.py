#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import argparse
import json
import logging
import math
import signal
import sys
import threading
import time

from monodrive import Simulator
from monodrive.configuration import SimulatorConfiguration, VehicleConfiguration
from monodrive.networking import messaging
from monodrive.sensors import GPS
from monodrive.networking.client import Client


LOG_CATEGORY = "test"

millis = lambda: int(round(time.time() * 1000))


class SensorTask:
    def __init__(self, sensor, clock_mode):
        self.sensor = sensor
        self.clock_mode = clock_mode
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

            self.data_received.set()

        try:
            seconds = (get_time() - start) / 1000
            fps = packets / seconds

            logging.getLogger(LOG_CATEGORY).info(
                '{0: >18}: {1:5.2f} fps ({2:3} frames received in {3:6.2f} secs ({4}))'.format(
                    self.sensor.name, fps, packets, seconds, time_units()))
        except:
            pass

    def stop(self):
        #logging.getLogger(LOG_CATEGORY).info("stopping %s" % self.sensor.name)
        self.sensor.stop()
        self.running = False

    def join(self):
        if self.thread is not None:
            self.thread.join()


def shutdown(sig, frame):
    time.sleep(5)
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, shutdown)

    simulator_config = SimulatorConfiguration('simulator.json')
    vehicle_config = VehicleConfiguration('test.json')

    client = Client((simulator_config.configuration["server_ip"], simulator_config.configuration["server_port"]))

    if not client.isconnected():
        client.connect()

    simulator = Simulator(client, simulator_config)

    sensor_configuration = vehicle_config.sensor_configuration
    vehicle_config.sensor_configuration = []
    vehicle_config.configuration['sensors'] = []

    logging.getLogger(LOG_CATEGORY).debug(json.dumps(vehicle_config.configuration))
    simulator.send_configuration()
    simulator.send_vehicle_configuration(vehicle_config)

    sensors = []
    idx = 0
    for sensor_config in sensor_configuration:
        cmd = messaging.AttachSensorCommand('ego', sensor_config)
        logging.getLogger(LOG_CATEGORY).debug('--> '.format(cmd))
        response = client.request(cmd)
        logging.getLogger(LOG_CATEGORY).debug('<-- '.format(response))

        sensor_class = vehicle_config.get_class(sensor_config['type'])
        sensors.append(sensor_class(idx, sensor_config, simulator_config))
        idx = idx + 1

    tasklist = []
    for sensor in sensors:
        sensor.start()
        sensor.send_start_stream_command(simulator)

    for sensor in sensors:
        try:
            st = SensorTask(sensor, vehicle_config.clock_mode)
            tasklist.append(st)
        except Exception as e:
            print(str(e))

    for sensor in sensors:
        sensor.socket_ready_event.wait()

    time.sleep(30)

    for task in tasklist:
        task.sensor.stop_sensor_command(client)
        task.stop()

    for sensor in sensors:
        sensor.join()

    for task in tasklist:
        task.join()

    simulator.stop()