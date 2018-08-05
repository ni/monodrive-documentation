
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import numpy as np
from matplotlib import pyplot as plt
from multiprocessing import Process, Manager
import time

from monodrive.networking import messaging, queues
from monodrive.sensors import MatplotlibSensorUI


class Map(MatplotlibSensorUI):

    def __init__(self, map_result, **kwargs):
        super(Map, self).__init__(**kwargs)

        self.name = 'Map'
        self.thread_state = Manager().Value('i', 0)
        self.main_plot = None

        self._data = queues.SingleQueue()
        self._local_data = MapData(map_result.data)
        self._data.put(self._local_data)

        self._map_subplot = None
        self._map_subplot_handle = None
        self._xy_combined = None

        if self._local_data.success:
            print("map: r:{0} l0:{1}".format(self.num_roads(), self.num_lanes(0)))

    @property
    def _map_data(self):
        if self._local_data is None:
            self._local_data = self._data.peek()
        return self._local_data

    def num_roads(self):
        return len(self._map_data.roads)

    def road(self, road_index):
        return MapData(self._map_data.roads[road_index])

    def num_lanes(self, road_index):
        return len(self.road(road_index).lanes)

    def lane(self, road_index, lane_index):
        return MapData(self.road(road_index).lanes[lane_index])

    def start(self):
        render_process = Process(target=self.rendering_main,
                                 name='Map_Render')
        render_process.daemon = True
        render_process.start()

    def stop(self):
        self.thread_state = 0

    @staticmethod
    def process_data_loop(a_map):
        a_map.thread_state = 1
        while a_map.thread_state == 1:
            time.sleep(0.1)
        a_map.thread_state = -1

    def initialize_views(self):
        self.view_lock.acquire()
        super(Map, self).initialize_views()

        self._map_subplot = self.main_plot.add_subplot(111)
        self._map_subplot_handle, = self._map_subplot.plot(0, 0, marker='.', linestyle='None')
        self._map_subplot.set_title("Ground Truth Map View")

        points_by_lane = []

        for road_index in range(0, self.num_roads()):
            for lane_index in range(0, self.num_lanes(road_index)):
                lane = self.lane(road_index, lane_index)
                lane_points = lane.points

                x = list(map(lambda p: p['x'] / 100, lane_points))
                y = list(map(lambda p: -p['y'] / 100, lane_points))

                xy_points = np.column_stack((x, y))
                points_by_lane.append(xy_points)

        x_combined = []
        y_combined = []
        for points in points_by_lane:
            x_combined = np.append(x_combined, points[:, 0])
            y_combined = np.append(y_combined, points[:, 1])

        self.display(x_combined, y_combined)

        self.view_lock.release()

    def display(self, x, y):
        self._map_subplot_handle.set_xdata(x)
        self._map_subplot_handle.set_ydata(y)
        margin = 10
        plt.axis((min(x) - margin, max(x) + margin, min(y) - margin, max(y) + margin))

    @property
    def window_configuration_coordinates(self):
        return '800x800+' + str(self.window_x_position) + '+' + str(self.window_y_position)

class MapData(object):
    def __init__(self, d):
        self.__dict__ = d