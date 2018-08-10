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
        self.waypoint = None

    def drive(self, sensors):
        self.waypoint_sensor = Waypoint.get_sensor(self.sensors)
        self.gps_sensor = GPS.get_sensor(self.sensors)
        print("mapping")
        self.mapping()
        print("perception")
        #self.perception()
        print("planning")
        move_velocity = self.planning(self.gps_sensor.world_location, self.gps_sensor.speed)
        print("control")
        control = self.control(move_velocity)
        return control

    def mapping(self):
        if self.waypoint_sensor:
            msg = self.waypoint_sensor.get_message()
            self.waypoint = Waypoint_Message(msg)

    def perception(self):
        if self.gps_sensor:
            msg = self.gps_sensor.get_message()
            self.gps = GPS_Message(msg)

    def planning(self, gps_location, vehicle_speed):
        """
        Using vehicle speed and location to find target point
        PURE_PURSUIT
        """

        target_lane = self.waypoint.lane_number
        target_waypoints = self.get_waypoints_by_lane(target_lane)

        cx = target_waypoints[:, 0]
        cy = target_waypoints[:, 1]

        estimated_ego_lane = self.get_estimated_current_lane(gps_location)
        idx, current_waypoint = self.find_closest_waypoint(gps_location, target_lane)
        self.update_tracking_index(idx, estimated_ego_lane, self.simulator)

        # Calculate look ahead distance based on speed and look forward gain
        look_ahead_distance = k * vehicle_speed + Lfc

        dif = current_waypoint - gps_location[:2:]

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

        dt = self.waypoint_sensor.game_time / 1000.0 - self.last_time
        self.last_time = self.waypoint_sensor.game_time / 1000.0

        # Find the difference from the current gps location and the target point
        dif = np.subtract(target_point, self.gps_sensor.world_location)

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
        forward = np.dot(self.gps_sensor.forward_vector, forward_intention)
        right = np.cross(self.gps_sensor.forward_vector, forward_intention)[2]  # get z vector for rotation

        if drive_vehicle:
            return {
                'forward': forward,
                'right': right,
            }
        else:
            return {
                'forward': 0.0,
                'right': 0.0,
            }
    
    #TODO this is wonky, need to fix this simulator instance access
    def update_tracking_index(self, tracking_point_index, ego_lane, simulator):
        update = 0
        if tracking_point_index > len(self.get_waypoints_for_current_lane()) / 2 and not bool(self.update_command_sent.value):
            #self.update_command_sent.value = True
            self.previous_points = self.get_waypoints_for_current_lane()
            update = int(len(self.get_waypoints_for_current_lane()) / 2)
            msg = messaging.WaypointUpdateCommand(tracking_point_index, ego_lane)
            simulator.request(msg)
        return update

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


    def get_waypoints_by_lane(self, lane):
        return self.points_by_lane[lane]

    def get_waypoints_for_current_lane(self):
        return self.get_waypoints_by_lane(lane_number)

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
