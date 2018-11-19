__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import logging
import math
import numpy as np

from . import BaseVehicle
from monodrive.sensors import Waypoint, GPS
from monodrive.models import Waypoint_Message, GPS_Message
from monodrive.networking import messaging


class LV_Vehicle(BaseVehicle):
    def __init__(self, simulator_config, vehicle_config , map=None, restart_event=None, **kwargs):
        super(LV_Vehicle, self).__init__(simulator_config, vehicle_config, restart_event)
        self.waypoint_sensor = None
        self.gps_sensor = None
        self.gps_msg = None
        self.waypoint_msg = None
        self.gps_location = None
        self.velocity = None
        self.last_time = 0

    def zero_control(self):
        return {
            'forward': 0.0,
            'right': 0.0,
        }

    def drive(self, sensors):
        self.sensors = sensors
        if not self.mapping():
            return self.zero_control()
        move_velocity = self.planning(self.gps_sensor.speed)
        control = self.control(move_velocity)
        return control


    def mapping(self):
        for sensor in self.sensors:
            # we grab all sensors here to garantee we keep the queues clean with fresh data

            msg = sensor.get_message(timeout=2)
            if sensor.__class__ == GPS:
                self.gps_msg = GPS_Message(msg)
                self.gps_location = [self.gps_msg.lat, self.gps_msg.lng]
                self.world_location = self.gps_msg.world_location
                self.velocity = self.gps_msg.speed
                self.gps_sensor = sensor
            elif sensor.__class__ == Waypoint:
                self.waypoint_msg = Waypoint_Message(msg)
                self.waypoint_sensor = sensor

        if self.waypoint_msg and self.gps_msg:
            return True
        else:
            return False

    def perception(self):
        pass

    def planning(self, vehicle_speed):

        return vehicle_speed

    def control(self, move_velocity):

        if drive_vehicle:
            return {
                'forward': forward,
                'right': right,
            }
        else:
            self.zero_control()