#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

from monodrive.ui.guiincludes import *


class RoadMap_View(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BACKGROUND_COLOR)
        self.figure = Figure()
        self.canvas = FigureCanvas(self, 1, self.figure)
        self.map_subplot = self.figure.add_subplot(111)
        #self.toolbar = NavigationToolbar(self.canvas)
        #self.toolbar.Realize()

        #this seems hacky but it is the only way to start the prot
        self.map_subplot_handle = None
        self.map_subplot.set_title("Ground Truth Map View")

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.canvas, 1,  wx.EXPAND)
        #self.sizer.Add(self.toolbar, 0, wx.LEFT | wx.EXPAND)
        self.SetSizer(self.sizer)
        pub.subscribe(self.update_view, "update_roadmap")

        self.road_map = None

    @property
    def map_data(self):
        return self.road_map

    def num_roads(self):
        return len(self.map_data.roads)

    def road(self, road_index):
        return MapData(self.map_data.roads[road_index])

    def num_lanes(self, road_index):
        return len(self.road(road_index).lanes)

    def lane(self, road_index, lane_index):
        return MapData(self.road(road_index).lanes[lane_index])

    def update_view(self, msg):
        self.road_map = MapData(msg)
        #if self.road_map:
        #pass

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

        self.update_plot(x_combined, y_combined)

    def update_plot(self, x, y):
        if self.map_subplot_handle == None:
            self.map_subplot_handle = self.map_subplot.plot(x, y, marker='.', linestyle='None')[0]
        self.map_subplot_handle.set_xdata(x)
        self.map_subplot_handle.set_ydata(y)
        self.figure.canvas.draw()
        #self.Layout()
        self.Refresh()
