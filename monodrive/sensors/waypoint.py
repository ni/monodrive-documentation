
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import numpy as np
import struct
import math
from multiprocessing import Value

from . import BaseSensor
from monodrive.networking import messaging


class Waypoint(BaseSensor):
    def __init__(self, idx, config, simulator_config, **kwargs):
        super(Waypoint, self).__init__(idx=idx, config=config, simulator_config=simulator_config, **kwargs)
        self.total_points = self.config['total_points']
        self.points_by_lane = None
        self.lane_number = None
        self.speed_limit_by_lane = None
        self.update_command_sent = Value('i', False)
        self.previous_points = None
        self.xy_combined = np.column_stack(([0], [0]))

    @classmethod
    def parse_frame(cls, frame, time_stamp, game_time):
        # frame = frame[0]
        fmt = '=iii'
        header_offset = 12
        cur_lane, total_points, total_lanes = list(struct.unpack(fmt, frame[:header_offset]))
        count = 2 * total_points * total_lanes
        fmt = '<' + str(count) + 'f'
        points = struct.unpack(fmt, frame[header_offset:count * 4 + header_offset:])
        speed_limits = struct.unpack('<' + str(total_lanes) + 'f', frame[header_offset + (count * 4):])
        speed_limit_by_lane = list(speed_limits)

        points_by_lane = []
        for lane_count in range(0, total_lanes):
            lane_points = points[lane_count * total_points * 2:(lane_count + 1) * total_points * 2:]
            n_points = np.array(lane_points)
            x = np.divide(n_points[0::2], 100)
            y = np.divide(n_points[1::2], 100)
            xy_points = np.column_stack((x, y))
            points_by_lane.append(xy_points)

        data_dict = {
            'time_stamp': time_stamp,
            'game_time': game_time,
            'points_by_lane': points_by_lane,
            'speed_limit_by_lane': speed_limit_by_lane,
            'lane_number': cur_lane
        }
        return data_dict

    def get_message(self, timeout = None):
        data = super(Waypoint, self).get_message(timeout = timeout)
        if self.update_command_sent is True:
            n1 = self.get_waypoints_for_current_lane()[0]
            try:
                p1 = self.previous_points[0]
            except:
                print("Waypoint Sensor is not working")
                p1 = None
            if not np.array_equal(n1, p1):
                self.update_command_sent.value = False

        return data

    '''def process_display_data(self):

        msg = self.q_data.get()

        self.view_lock.acquire()
        points_by_lane = msg['points_by_lane']
        x_combined = []
        y_combined = []
        for points in points_by_lane:
            x_combined = np.append(x_combined, points[:, 0])
            y_combined = np.append(y_combined, points[:, 1])

        self.xy_combined = np.column_stack((x_combined, y_combined))

        self.view_lock.release()
        self.update_sensors_got_data_count()'''


    '''def update_tracking_index(self, tracking_point_index, ego_lane):
        msg = messaging.WaypointUpdateCommand(tracking_point_index, ego_lane)
        self.simulator.request(msg)

    def update_tracking_index(self, tracking_point_index, ego_lane, simulator):
        update = 0
        if tracking_point_index > len(self.get_waypoints_for_current_lane()) / 2 and not bool(self.update_command_sent.value):
            self.update_command_sent.value = True
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
        return ind, current_lane_waypoints[ind]'''

    # GETTERS



    '''def get_lane(self):
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

        return dif_by_lane.index(min(dif_by_lane))'''

