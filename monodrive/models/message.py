#!/usr/bin/env python

import random
import time
import math
import numpy as np
import cPickle as pickle

class IMU_Message(object):
    def __init__(self, msg):
        self.string_accel_x = msg['acceleration_vector'][0]
        self.string_accel_y = msg['acceleration_vector'][1]
        self.string_accel_z = msg['acceleration_vector'][2]
        self.string_ang_rate_x = msg['angular_velocity_vector'][0]
        self.string_ang_rate_y = msg['angular_velocity_vector'][1]
        self.string_ang_rate_z = msg['angular_velocity_vector'][2]
        self.game_time_stamp = msg['time_stamp']
        self.time_stamp = msg['time_stamp']
    
    @classmethod
    def test_message(self):
        msg_list = []
        for x in range(7):
            msg_list.append(random.random())
        acceleration_vector = [msg_list[0], msg_list[1], msg_list[2]]
        angular_velocity_vector = [msg_list[3], msg_list[4], msg_list[5]]
        msg_dict = {
            'time_stamp': msg_list[6],
            'game_time': time.time(),
            'acceleration_vector': acceleration_vector,
            'angular_velocity_vector': angular_velocity_vector,
            'timer': time.time()
        }
        return msg_dict

class Waypoint_Message(object):
    def __init__(self, msg):
        self.time_stamp = msg['time_stamp']
        self.game_time = msg['game_time']
        self.points_by_lane = msg['points_by_lane']
        self.speed_limit_by_lane = msg['speed_limit_by_lane']
        self.lane_number = msg['lane_number']

    @classmethod
    def test_message(self):
        msg_list = []
        for x in range(12):
            msg_list.append(random.random(1,1000))

        msg_dict = {
            'time_stamp': time.time(),
            'game_time': time.time(),
            'points_by_lane': msg_list[0],
            'speed_limit_by_lane': 30,
            'lane_number': 1
        }    

        return msg_dict
   
class GPS_Message(object):
    def __init__(self, msg):
        self.lat = msg['lat']
        self.lng = msg['lng']
        self.time_stamp = msg['time_stamp']

    @classmethod
    def test_message(self):
        msg_list = []
        for x in range(12):
            msg_list.append(random.random(1,1000))
        forward_vector = np.array([msg_list[0], msg_list[1], msg_list[2]])
        world_location = np.array([msg_list[3] / 100.0, msg_list[4] / 100.0, 0.0])
        msg_dict = {
            'time_stamp': msg_list[5],
            'game_time': msg_list[6],
            'lat': msg_list[7],
            'lng': msg_list[8],
            'elevation': msg_list[9],
            'forward_vector': forward_vector,
            'world_location': world_location,
            'ego_yaw': msg_list[10],
            'speed': msg_list[11]
        }
        return msg_dict

class Camera_Message(object):
      
    def __init__(self, msg):
        self.np_image = np.array(pickle.loads(msg))
    
    def test_message(self):
        self.np_image = None

class MapData(object):
    def __init__(self, d):
        self.__dict__ = d
