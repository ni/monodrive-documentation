#!/usr/bin/env python

import random
import time

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
