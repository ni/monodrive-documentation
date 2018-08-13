__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import math
import numpy as np

from . import BaseVehicle
from monodrive.sensors import Waypoint, GPS
from monodrive.models import Waypoint_Message, GPS_Message
from monodrive.networking import messaging 


k = 1.6  # look forward gain, dependent on vehicle speed
Lfc = 5.0  # look-ahead distance
max_vel = .3  # Max velocity
max_turn_change = .1
drive_vehicle = True


class SimpleVehicle(BaseVehicle):
    def __init__(self, simulator_config, vehicle_config, map=None, restart_event=None, **kwargs):
        super(SimpleVehicle, self).__init__(simulator_config, vehicle_config, restart_event)
        self.waypoint_sensor = None
        self.gps_sensor = None
        self.gps_msg = None
        self.waypoint_msg = None
        self.gps_location = None
        self.velocity = None

    def zero_control(self):
        return {
                'forward': 0.0,
                'right': 0.0,
            }
    def drive(self, sensors):
        self.sensors = sensors
        if not self.mapping_with_multimessges():
            return self.zero_control()
        move_velocity = self.planning(self.gps_sensor.speed)
        control = self.control(move_velocity)
        return control

    def mapping_with_multimessges(self):
        self.gps_msg = None
        self.waypoint_msg = None
        for sensor in self.sensors:
            n = sensor.get_frame_count()
            if n > 0:
                messages = sensor.get_messages()
                msg = messages.pop()  #throws away older messages
                if sensor.__class__ == GPS:
                    self.gps_msg = GPS_Message(msg)
                    self.gps_location = [self.gps_msg.lat, self.gps_msg.lng]
                    self.world_location = self.gps_msg.world_location
                    self.velocity = self.gps_msg.speed
                    self.gps_sensor = sensor
                    #print("vehicle{:>26} ts={:>10} gt={:>10} messages={:>3}".format(sensor.name, self.gps_msg.time_stamp, 
                    #                                                                                self.gps_msg.game_time,n)) 
                elif sensor.__class__ == Waypoint:   
                    self.waypoint_msg = Waypoint_Message(msg) 
                    self.waypoint_sensor = sensor
                    #print("vehicle{:>26} ts={:>10} gt={:>10} messages={:>3}".format(sensor.name, self.waypoint_msg.time_stamp, 
                    #                                                                                self.waypoint_msg.game_time, n)) 
                sensor.message_event.clear()

        if self.waypoint_msg and self.gps_msg:
            return True
        else:
            print("missing required sensor frames")     
            return False 

            

    def mapping_with_pipe(self):
        #print("mapping_with_pipe")
        for sensor in self.sensors:
            try:
                sensor.message_event.wait(timeout = .1)
            except Exception as e:
                print("{0} message event error: {1}".format(sensor.name, e))

        for sensor in self.sensors:
            msg = sensor.rx_pipe.recv()
            #print("{0} msg length = {1}".format(sensor.name, len(str(msg))))
            if sensor.__class__ == GPS:
                self.gps_msg = GPS_Message(msg)
                self.gps_location = [self.gps_msg.lat, self.gps_msg.lng]
                self.world_location = self.gps_msg.world_location
                self.velocity = self.gps_msg.speed
                self.gps_sensor = sensor
                print("vehicle{:>26} ts={:>10} gt={:>10} msg_len{:>8}".format(sensor.name, self.gps_msg.time_stamp, self.gps_msg.game_time, len(str(msg)))) 
            elif sensor.__class__ == Waypoint:   
                self.waypoint_msg = Waypoint_Message(msg) 
                self.waypoint_sensor = sensor
                print("vehicle{:>26} ts={:>10} gt={:>10} msg_len{:>8}".format(sensor.name, self.gps_msg.time_stamp, self.gps_msg.game_time,len(str(msg)))) 
            sensor.message_event.clear()

        if self.waypoint_msg and self.gps_msg:
            return True
        else:
            print("missing required sensor frames")     
            return False 
                
    def mapping(self):
        for sensor in self.sensors:
            #we grab all sensors here to garantee we keep the queues clean with fresh data

            msg = sensor.get_message(timeout = 2)
            if sensor.__class__ == GPS:
                self.gps_msg = GPS_Message(msg)
                self.gps_location = [self.gps_msg.lat, self.gps_msg.lng]
                self.world_location = self.gps_msg.world_location
                self.velocity = self.gps_msg.speed
                self.gps_sensor = sensor
                print("vehicle{:>26} ts={:>10} gt={:>10} msg_len{:>8}".format(sensor.name, self.gps_msg.time_stamp, self.gps_msg.game_time, len(str(msg)))) 
            elif sensor.__class__ == Waypoint:   
                self.waypoint_msg = Waypoint_Message(msg) 
                self.waypoint_sensor = sensor
                print("vehicle{:>26} ts={:>10} gt={:>10} msg_len{:>8}".format(sensor.name, self.gps_msg.time_stamp, self.gps_msg.game_time,len(str(msg)))) 
            
        if self.waypoint_msg and self.gps_msg:
            return True
        else:
            print("missing required sensor frames")     
            return False      

    def perception(self):
        pass


    def planning(self, vehicle_speed):
        """
        Using vehicle speed and location to find target point
        PURE_PURSUIT
        """

        target_lane = self.waypoint_msg.lane_number
        target_waypoints = self.waypoint_msg.get_waypoints_by_lane(target_lane)

        cx = target_waypoints[:, 0]
        cy = target_waypoints[:, 1]

        estimated_ego_lane = self.waypoint_msg.get_estimated_current_lane(self.world_location)
        idx, current_waypoint = self.waypoint_msg.find_closest_waypoint(self.world_location, target_lane)
        
        if idx > len(self.waypoint_msg.get_waypoints_for_current_lane()) / 2: #and not bool(self.update_command_sent.value):
            #self.update_command_sent.value = True
            self.previous_points = self.waypoint_msg.get_waypoints_for_current_lane()
            #update = int(len(self.waypoint_msg.get_waypoints_for_current_lane()) / 2)
            msg = messaging.WaypointUpdateCommand(idx, target_lane)
            self.simulator.request(msg)

        #self.waypoint_sensor.update_tracking_index(idx, estimated_ego_lane, self.simulator)

        # Calculate look ahead distance based on speed and look forward gain
        look_ahead_distance = float(k) * self.velocity + Lfc

        dif = current_waypoint - self.world_location[:2:]

        L = np.linalg.norm(dif)

        # Find target point by searching distances from ego up to look ahead distance
        while look_ahead_distance > L and (idx + 1) < len(cx):
            dx = cx[idx + 1] - cx[idx]
            dy = cy[idx + 1] - cy[idx]
            L += math.sqrt(dx ** 2 + dy ** 2)
            idx += 1

        target_point = target_waypoints[idx]
        target_point = np.append(target_point, [0.0])
        # make 2d ad 3d vector

        dt = self.waypoint_msg.game_time / 1000.0 - self.last_time
        self.last_time = self.waypoint_msg.game_time / 1000.0

        # Find the difference from the current gps location and the target point
        dif = np.subtract(target_point, self.world_location)

        # Compute move velocity vector based on difference in locations over change in time
        move_velocity = dif / dt
        return move_velocity

    def control(self, move_velocity):
        # Calculate forward and right control values based on velocity and forward vectors
        """
        Vector math to cFonvert move_velocity and forward_vector into forward and right intentions
        """

        mag = math.sqrt(move_velocity[0] ** 2 + move_velocity[1] ** 2 + move_velocity[2] ** 2)
        if mag == 0.0:
            return 0.0, 0.0
        norm = [move_velocity[0] / mag, move_velocity[1] / mag, move_velocity[2] / mag]
        forward_intention = np.array(norm)
        forward = np.dot(self.gps_msg.forward_vector, forward_intention)
        right = np.cross(self.gps_msg.forward_vector, forward_intention)[2]  # get z vector for rotation

        if drive_vehicle:
            return {
                'forward': forward,
                'right': right,
            }
        else:
            self.zero_control()
