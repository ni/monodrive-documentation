#!/usr/bin/env python

import random
import time
import math
import numpy as np
try:
    import cPickle as pickle
except:
    import _pickle as pickle

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


    def find_closest_waypoint(self, gps_location, lane):
        current_lane_waypoints = self.get_waypoints_by_lane(lane)
        cx = current_lane_waypoints[:, 0]
        cy = current_lane_waypoints[:, 1]

        # Find distances between vehicle location and each pooint
        dx = [gps_location[0] - icx for icx in cx]
        dy = [gps_location[1] - icy for icy in cy]

        # Magnitude of distances
        d = [math.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]

        # Index of minimum distance
        ind = d.index(min(d))
        return ind, current_lane_waypoints[ind]

    # GETTERS

    def get_lane(self):
        return self.lane_number

    def get_waypoints_by_lane(self, lane):
        return self.points_by_lane[lane]

    def get_waypoints_for_current_lane(self):
        return self.get_waypoints_by_lane(self.lane_number)

    def get_waypoint_in_lane(self, lane, index):
        lane_waypoints = self.get_waypoints_by_lane(lane)
        return lane_waypoints[index]

    def get_waypoint_in_current_lane(self, index):
        self.get_waypoint_in_lane(self.lane_number, index)

    def get_estimated_current_lane(self, gps_location):
        dif_by_lane = []
        for lane in range(0, len(self.points_by_lane)):
            ind, wp = self.find_closest_waypoint(gps_location, lane)
            dif = wp - gps_location[:2:]
            mag = math.sqrt(dif[0] ** 2 + dif[1] ** 2)
            dif_by_lane.append(mag)

        return dif_by_lane.index(min(dif_by_lane))

   
class GPS_Message(object):
    def __init__(self, msg):
        self.time_stamp = msg['time_stamp']
        self.game_time = msg['game_time']
        self.lat = msg['lat']
        self.lng = msg['lng']
        self.elevation = msg['elevation']
        self.forward_vector = msg['forward_vector']
        self.world_location = msg['world_location']
        self.ego_yaw = msg['ego_yaw']
        self.speed = msg['speed']

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
        self.np_image = np.array(msg)
    
    def test_message(self):
        self.np_image = None

class MapData(object):
    def __init__(self, d):
        self.__dict__ = d
