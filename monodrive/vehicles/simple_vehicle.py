__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import math
import numpy as np

from . import BaseVehicle
from monodrive.sensors import Waypoint, GPS


k = 1.6  # look forward gain, dependent on vehicle speed
Lfc = 5.0  # look-ahead distance
max_vel = .3  # Max velocity
max_turn_change = .1
drive_vehicle = True


class SimpleVehicle(BaseVehicle):
    def __init__(self, simulator_config, vehicle_config, restart_event=None, **kwargs):
        super(SimpleVehicle, self).__init__(simulator_config, vehicle_config, restart_event)
        self.waypoint_sensor = Waypoint.get_sensor(self.sensors)
        self.gps_sensor = GPS.get_sensor(self.sensors)

    def drive(self, sensors, vehicle_state):
        self.mapping()
        self.perception()
        move_velocity = self.planning(self.gps_sensor.world_location, self.gps_sensor.speed)
        return self.control(move_velocity)

    def mapping(self):
        if self.waypoint_sensor:
            self.waypoint_sensor.get_message()

    def perception(self):
        if self.gps_sensor:
            self.gps_sensor.get_message()

    def planning(self, gps_location, vehicle_speed):
        """
        Using vehicle speed and location to find target point
        PURE_PURSUIT
        """

        target_lane = BaseVehicle.plan_target_lane(self.waypoint_sensor, self.vehicle_state)
        target_waypoints = self.waypoint_sensor.get_waypoints_by_lane(target_lane)

        cx = target_waypoints[:, 0]
        cy = target_waypoints[:, 1]

        estimated_ego_lane = self.waypoint_sensor.get_estimated_current_lane(gps_location)
        idx, current_waypoint = self.waypoint_sensor.find_closest_waypoint(gps_location, target_lane)
        self.waypoint_sensor.update_tracking_index(idx, estimated_ego_lane, self.simulator)

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
        Vector math to convert move_velocity and forward_vector into forward and right intentions
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
