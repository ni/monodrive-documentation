
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import math
import numpy as np
import sys

from . import BaseVehicle


k = 1.6  # look forward gain, dependent on vehicle speed
Lfc = 12.0  # look-ahead distance
max_vel = .6  # Max velocity
max_turn_change = .1


def calc_target_index(vehicle_location, vehicle_speed, points):
    """
    Using vehicle speed and location to find target point
    """
    # Split x and y values of all the points
    cx = points[:, 0]
    cy = points[:, 1]

    # Find distances between vehicle location and each pooint
    dx = [vehicle_location[0] - icx for icx in cx]
    dy = [vehicle_location[1] - icy for icy in cy]

    # Magnitude of distances
    d = [math.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]

    # Index of minimum distance
    ind = d.index(min(d))

    # Calculate look ahead distance based on speed and look forward gain
    look_ahead_distance = k * vehicle_speed + Lfc

    dif = points[ind] - vehicle_location[:2:]
    L = np.linalg.norm(dif)

    # Find target point by searching distances from ego up to look ahead distance
    while look_ahead_distance > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
        L += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1
    return ind


def check_front_traffic(data, forward, right, vehicle_speed):
    x_points = data['x_points']
    y_points = data['y_points']
    velocities = data['velocities']

    if np.size(x_points) == 0:
        return forward, right

    look_ahead = 1.7*vehicle_speed + 35
    cars_in_lane_x_indices = np.where(np.logical_and(x_points >= -2, x_points <= 2))
    cars_in_lanes_y_values = np.take(y_points, cars_in_lane_x_indices)
    #print('vel {0} ahead {1} y {2} vel {3}'.format(vehicle_speed, look_ahead, cars_in_lanes_y_values, velocities))
    cars_in_front_lane_y_indices = np.where(np.logical_and(cars_in_lanes_y_values >= 0.0, cars_in_lanes_y_values <= look_ahead))

    velocity_car_front, y_value_car_front = None, None
    if np.size(cars_in_front_lane_y_indices) > 0:
        min_y_index = np.argmin(cars_in_front_lane_y_indices)

        velocity_car_front = velocities[min_y_index]
        y_value_car_front = y_points[min_y_index]
        # print('front velocity y ', velocity_car_front, y_value_car_front)

    AccelerationScaling = 25
    maxAcceleration = .7
    desired_speed = 60
    kp = proportional_gain = .2
    kd = derivative_gain = .4

    dr = relative_distance = velocity_car_front
    dd = desired_distance = 25
    if velocity_car_front is not None and velocity_car_front < 0.0:
        p = relative_distance - desired_distance
        vr = relative_velocity = vehicle_speed - velocity_car_front
        # print('slow down ', p, vr)
    else:
        p = 0
        vr = desired_speed - vehicle_speed

    acceleration = kp * p + kd * vr
    acceleration = acceleration / AccelerationScaling;

    acceleration = min(maxAcceleration, acceleration)

    return acceleration, right


def accelerate(move_velocity, forward_vector):
    mag = math.sqrt(move_velocity[0] ** 2 + move_velocity[1] ** 2 + move_velocity[2] ** 2)
    if mag == 0.0:
        return 0.0, 0.0
    norm = [move_velocity[0] / mag, move_velocity[1] / mag,  move_velocity[2] / mag]
    forward_intention = np.array(norm)
    p_velocity = np.dot(forward_vector, forward_intention)
    right_throw = np.cross(forward_vector, forward_intention)[2] / 2
    # p_velocity = np.clip(p_velocity, 0.2, max_vel)
    return p_velocity, right_throw


class MDVehicle(BaseVehicle):
    def drive(self, sensors, vehicle_state):
        waypoint_q = None
        gps_q = None

        for s in sensors:
            if s.type == 'Waypoint':
                waypoint_q = s.q_vehicle
            if s.type == 'GPS':
                gps_q = s.q_vehicle

        # Send forward command to move car so sensors can see data
        if waypoint_q is None:
            return {
                'forward': 1.0,
                'right': 0.0,
                'update_waypoints': 0,
                'current_lane': 0,
                'restart': False,
                'offset': 0,
                'speed_limit': 3333
            }

        data_gps = gps_q.get()
        data_waypoint = waypoint_q.get()


        points_by_lane = data_waypoint['points_by_lane']
        cur_lane = data_waypoint['current_lane']
        speed_limit = data_waypoint['speed_limit_by_lane'][cur_lane]

        new_points = points_by_lane[cur_lane]
        if self.update_sent:
            n1 = new_points[0]
            p1 = self.points[0]
            if not np.array_equal(n1, p1):
                update_sent = False

        self.points = new_points

        # Calculate Velocity and forward vector
        speed = data_gps['speed']
        forward_vector = np.array(data_gps['forward_vector'])

        # Get target point from velocity, gps loc, and points
        gps_loc = np.array([data_gps['world_location'][0], data_gps['world_location'][1], 0.0])
        target_index = calc_target_index(gps_loc, speed, self.points)
        target_point = np.append(self.points[target_index], [0.0])

        time = data_waypoint['time_stamp']
        time = time / 1000.0
        if self.last_time == 0.0:
            self.last_time = time
            return {
                'forward': 0.0,
                'right': 0.0,
                'update_waypoints': 0,
                'current_lane': 0,
                'restart': False,
                'offset': self.get_lane_offset(gps_loc),
                'speed': speed,
                'speed_limit': speed_limit
            }

        dt = time - self.last_time
        self.last_time = time
        if dt == 0.0:
            return {
                'forward': 0.0,
                'right': 0.0,
                'update_waypoints': 0,
                'current_lane': 0,
                'restart': False,
                'offset': self.get_lane_offset(gps_loc),
                'speed': speed,
                'speed_limit': speed_limit
            }

        # Simplfy ELI5
        dif = np.subtract(target_point, gps_loc)
        move_velocity = dif / dt
        forward, right = accelerate(move_velocity, forward_vector)

        # print('x and y', abs(gps_loc[0] - 175.0), gps_loc, target_point)
        # forward, right = check_front_traffic(data_bounding, forward, right, speed)

        truck_location_x = 213.0
        if abs(gps_loc[0] - truck_location_x) < 80.0:
            forward = forward * abs(gps_loc[0] - truck_location_x) / truck_location_x

        restart = False
        if self.restart_event and (gps_loc[0] - truck_location_x) < 30.0:
            self.restart_event.set()
            forward = 0.0
            restart = True

        update = 0
        if target_index > len(self.points) / 2 and not self.update_sent:
            self.update_sent = True
            update = int(len(self.points) / 2)

        return {
                'forward': forward,
                'right': right,
                'update_waypoints': update,
                'current_lane': cur_lane,
                'restart': restart,
                'offset': self.get_lane_offset(gps_loc),
                'speed': speed,
                'speed_limit': speed_limit
            }

    def get_lane_offset(self, current_position):
        min = sys.float_info.max
        pos = current_position[0:2]
        index = -1
        n = 0
        for point in self.points:
            dist = np.subtract(point, pos)
            dsq = dist[0] * dist[0] + dist[1] * dist[1]
            if dsq < min:
                a = point
                min = dsq
                index = n
            n = n + 1

        if index > 0:
            b = self.points[index - 1]
        else:
            b = self.points[index + 1]

        m = math.fabs(a[1] - b[1]) / (a[0] - b[0])
        print(current_position, a[0],a[1],b[0],b[1],m)
        if m == 0.0:
            return 0
        return math.fabs(m * current_position[0] + current_position[1]) / math.sqrt(m * m)
