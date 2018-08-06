
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import matplotlib
import numpy as np
import struct
import matplotlib.patches as patches

from . import BaseSensorPacketized
from .gui import MatplotlibSensorUI

matplotlib.use('TkAgg')

SHOW_MAP = True


class BoundingBox(MatplotlibSensorUI, BaseSensorPacketized):
    def __init__(self, idx, config, simulator_config, **kwargs):
        super(BoundingBox, self).__init__(idx=idx, config=config, simulator_config=simulator_config, **kwargs)
        #self.plot = None
        self.patches = []
        self.distances = None
        self.angles = None
        self.x_points = []
        self.y_points = []
        self.x_bounds = None
        self.y_bounds = None
        self.box_rotations = None
        self.velocities = None

    @classmethod
    def parse_frame(cls, frame, time_stamp, game_time):
        # frame = frame[0]
        fmt = '=hi'
        cur_frame_start = 0
        cur_frame_end = 6
        start_id, number_of_targets = list(struct.unpack(fmt, frame[cur_frame_start:cur_frame_end]))
        distances, angles, x_points, y_points, x_bounds, y_bounds, z_bounds, box_rotations, velocities, radar_distances,\
            radar_angles, radar_velocities = [], [], [], [], [], [], [], [], [], [], [], []
        bounding_box_positions, in_line_of_sight = {}, {}

        if number_of_targets > 0:
            fmt = '<' + (str(7) + 'f?') * number_of_targets
            bounding_box_data_bytes_count = number_of_targets * (7 * 4 + 1)
            cur_frame_start = cur_frame_end
            cur_frame_end = cur_frame_start + bounding_box_data_bytes_count
            targets = np.array(np.array_split(struct.unpack(fmt, frame[cur_frame_start:cur_frame_end]), number_of_targets))

            target_points = np.array([create_point(t[0], t[1]) for t in targets])
            distances = targets[:, 0]
            angles = np.multiply(targets[:, 1], -1)
            x_points = target_points[:, 0]
            y_points = target_points[:, 1]
            x_bounds = targets[:, 3]
            y_bounds = targets[:, 2]
            z_bounds = targets[:, 4]
            box_rotations = np.multiply(targets[:, 5], -1)
            velocities = targets[:, 6]
            in_radar_fov_points = np.array([t for t in targets if t[7]])
            if len(in_radar_fov_points):
                radar_distances = in_radar_fov_points[:, 0]
                radar_angles = np.multiply(in_radar_fov_points[:, 1], -1)
                radar_velocities = in_radar_fov_points[:, 6]

        fmt = '=i'
        cur_frame_start = cur_frame_end
        cur_frame_end = cur_frame_start+4
        number_of_cameras = list(struct.unpack(fmt, frame[cur_frame_start:cur_frame_end]))[0]

        for i in range(0, number_of_cameras):
            fmt = '=i'
            cur_frame_start = cur_frame_end
            cur_frame_end = cur_frame_start+4
            camera_id = str(list(struct.unpack(fmt, frame[cur_frame_start:cur_frame_end]))[0])

            number_of_floats = 16
            size_of_float = 4
            size_of_bool = 1
            fmt = '<' + (str(number_of_floats) + 'f?') * number_of_targets
            camera_box_bytes_count = number_of_targets * (number_of_floats * size_of_float + size_of_bool)
            cur_frame_start = cur_frame_end
            cur_frame_end = cur_frame_start + camera_box_bytes_count
            camera_boxes = np.array(struct.unpack(fmt, frame[cur_frame_start:cur_frame_end]))

            if number_of_targets > 0:
                camera_bounding_boxes = np.array(np.array_split(camera_boxes[:], number_of_targets))
                bounding_box_positions[camera_id] = camera_bounding_boxes[:, 0:16]
                in_line_of_sight[camera_id] = camera_bounding_boxes[:, 16]
            else:
                bounding_box_positions[camera_id] = []
                in_line_of_sight[camera_id] = []

        data_dict = {
            'time_stamp': time_stamp,
            'game_time': game_time,
            'distances': distances,
            'angles': angles,
            'x_points': x_points,
            'y_points': y_points,
            'x_bounds': x_bounds,
            'y_bounds': y_bounds,
            'z_bounds': z_bounds,
            'box_rotations': box_rotations,
            'velocities': velocities,
            'radar_distances': radar_distances,
            'radar_angles': radar_angles,
            'radar_velocities': radar_velocities,
            'bounding_box_positions': bounding_box_positions,

            'in_camera_los': in_line_of_sight
        }
        return data_dict

    def initialize_views(self):
        self.view_lock.acquire()
        super(BoundingBox, self).initialize_views()
        self.main_plot.set_size_inches(3.0,2.0)
        self.map_subplot = self.main_plot.add_subplot(111)
#        self.display(x_points, y_points, x_bounds, y_bounds, box_rotations)
        self.map_subplot.set_xlim(-80, 80)
        self.map_subplot.set_ylim(-80, 80)
        self.map_subplot.set_title("Vehicle Targets (birds eye)")
        self.map_subplot.set_xlabel('Range X (m)')
        self.map_subplot.set_ylabel('Range Y (m)')
        self.map_subplot.add_patch(patches.Rectangle((-1.0, -2.0), 2.0, 4.0, color='r'))
        self.view_lock.release()

    def display(self, x_points, y_points, x_bounds, y_bounds, box_rotations):
        for patch in self.patches:
            patch.remove()
        self.patches = []

        if len(x_points) == 0:
            #plt.pause(.001)
            return

        for i in range(0, len(x_points)):
            p = self.map_subplot.add_patch(patches.Rectangle((x_points[i], y_points[i]), x_bounds[i], y_bounds[i],
                                                             angle=box_rotations[i], color='b',fill=False))
            self.patches.append(p)
        #plt.pause(.001)

    def update_views(self, frame):
        if SHOW_MAP:
            self.view_lock.acquire()
            self.display(self.x_points, self.y_points, self.x_bounds, self.y_bounds, self.box_rotations)
            self.view_lock.release()
            return self.map_subplot
        return None

    def process_display_data(self):
        data = self.q_display.get()
        self.view_lock.acquire()
        self.x_points = data['x_points']
        self.y_points = data['y_points']
        self.x_bounds = data['x_bounds']
        self.y_bounds = data['y_bounds']
        self.box_rotations = data['box_rotations']
        self.view_lock.release()
        self.update_sensors_got_data_count()


def create_point(distance, degrees):
    theta = np.radians(degrees)
    c, s = np.cos(theta), np.sin(theta)
    x = -distance * s
    y = distance * c
    return np.array([x, y])
