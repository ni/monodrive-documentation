#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import wx
import wx.lib.mixins.inspection
from wx.lib.pubsub import pub

from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import NavigationToolbar2WxAgg as NavigationToolbar

try:
    import cPickle as pickle
except:
    import _pickle as pickle
#import matplotlib
#from matplotlib import pyplot as plt

#For image getting
from os import path
basepath = path.dirname(__file__)
filepath = path.abspath(path.join(basepath, "Capture.png"))

import numpy as np
from PIL import Image
import cv2
#f = open(filepath, "r")

import multiprocessing
from threading import Thread
try:
    import prctl
except: pass

import socket
import sys
import time

from monodrive.ui.buffered_window import BufferedWindow
from monodrive.constants import VELOVIEW_PORT, VELOVIEW_IP, ClockMode_ClientStep
from monodrive.models import IMU_Message
from monodrive.models import GPS_Message
from monodrive.models import Camera_Message
from monodrive.models import MapData
from monodrive.models import Radar_Message
from monodrive.models import Bounding_Box_Message


BACKGROUND_COLOR = '#eaf7ff'
INNER_PANEL_COLOR = '#f0f0f0'
BLACK = '#000000'
WHITE = '#ffffff'


class TextRow(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BACKGROUND_COLOR)


class GraphRow(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BACKGROUND_COLOR)


class CameraRow(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BACKGROUND_COLOR)


class Bounding_Polar_Plot(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BACKGROUND_COLOR)
        self.figure = Figure()
        self.canvas = FigureCanvas(self, 1, self.figure)
        #self.target_polar_subplot = self.figure.add_subplot(111, polar=True)
        self.target_polar_subplot = self.figure.add_axes([0, 0.07, 1, .8], polar=True)
        self.target_polar_subplot.set_title('Bounding Box Target Plot', pad=12)
        #self.target_polar_subplot.set_thetamin(-10)
        #self.target_polar_subplot.set_thetamax(10)
        self.target_polar_subplot.set_ylim(0, 150)
        #self.target_polar_subplot.set_
        self.target_polar_subplot.set_theta_zero_location('N')
        self.toolbar = NavigationToolbar(self.canvas)
        self.toolbar.Realize()
        pub.subscribe(self.update_view, 'update_bounding_box')

        N = 20
        r = 150 * np.random.rand(N)
        theta = 2 * np.pi * np.random.rand(N)
        #area = .01*r**2
        colors = theta

        #self.target_polar_handle = self.target_polar_subplot.scatter(theta, r, c=colors, s=area, cmap='hsv', alpha =0.75)
        self.target_polar_handle = self.target_polar_subplot.scatter(theta, r, marker='o', cmap='hsv', alpha =0.75)
        self.string_time = wx.StaticText(self, label="", style=wx.ELLIPSIZE_END)
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.string_time, 0,  wx.HORIZONTAL | wx.EXPAND)
        self.sizer.Add(self.canvas, 1,  wx.ALL | wx.EXPAND)
        self.sizer.Add(self.toolbar, 0,  wx.HORIZONTAL | wx.EXPAND)
        self.SetSizer(self.sizer)

    def update_view(self, msg):
        if msg:
            self.targets = Bounding_Box_Message(msg)
            self.update_plot(self.targets)
            self.string_time.SetLabelText('GAMETIME: {0: .2f} \tTIMESTAMP: {1}'.
                                          format(self.targets.game_time, self.targets.time_stamp))
        else:
            print("empty target list")

    def update_plot(self, targets):
        if len(targets.radar_distances) > 0:
            self.set_data(targets)
        #self.Layout()
        self.Refresh()

    def set_data(self, targets):
        r = targets.radar_distances
        theta = -np.radians(targets.radar_angles)
        #print("bounding_box distance = {0}".format(r))
        #print("bounding_box theta = {0}".format(theta))
        #rcs = targets.angles
        #speed = targets.velocities/100
        #there seems to be a bug in the new polar plot library, set_offsets is not working
        #so we have to do all the following on every frame
        self.target_polar_subplot.cla()
        self.target_polar_subplot.set_title('Bounding Box Target Plot')
        #self.target_polar_subplot.set_thetamin(-10)
        #self.target_polar_subplot.set_thetamax(10)
        #self.target_polar_subplot.set_rorigin(-40)
        #self.target_polar_subplot.set_ylim(40, 150)
        self.target_polar_subplot.set_theta_zero_location('N')
        self.target_polar_subplot.scatter(theta, r, c='b', cmap='hsv', alpha =0.75)
        #self.target_polar_handle.set_offsets([theta,r])
        self.figure.canvas.draw()


class Radar_FFT_Plot(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BACKGROUND_COLOR)
        self.figure = Figure()
        self.canvas = FigureCanvas(self, 1, self.figure)
        self.range_fft_subplot = self.figure.add_subplot(111)
        self.range_fft_subplot.set_title('Range_FFT')
        self.range_fft_subplot.set_ylim([-5,3])
        self.toolbar = NavigationToolbar(self.canvas)
        self.toolbar.Realize()

        #this seems hacky but it is the only way to start the prot
        self.range_fft_subplot_handle = None

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.canvas, 1,  wx.EXPAND)
        self.sizer.Add(self.toolbar, 0, wx.HORIZONTAL | wx.EXPAND)
        self.SetSizer(self.sizer)
        pub.subscribe(self.update_view, "update_radar_table")

    def update_view(self, msg):
        radar_msg = Radar_Message(msg)
        if radar_msg:
            range_fft = radar_msg.range_fft
            target_range_idx = radar_msg.target_range_idx
            self.update_range_fft_plot(range_fft, target_range_idx)


    def update_range_fft_plot(self, range_fft, target_range_idx):
        x = range(len(range_fft)/4)
        #print(target_range_idx)
        #if self.range_fft_subplot_handle == None:
        self.range_fft_subplot.cla()
        self.range_fft_subplot.set_title('Range by FFT (psd dBm)')

        #range_fft_power = 20.0 * np.log10(range_fft[0:len(x)])
        Fs = 150.0e6
        N = 1024.0
        psdx = np.absolute(range_fft)**2/(Fs*N)
        psdx = 2*psdx
        freq = range(0,int(Fs/8), int(Fs/len(x)/8))
        psdx_dbm = 10 * np.log10(psdx) - 30  #30 is db to dbm
        #fft_power_max = max(psdx_db)
        #fft_power_min = min(psdx_db)
        self.range_fft_subplot.set_ylim([-110, -40])
        self.range_fft_subplot_handle = self.range_fft_subplot.plot(freq[0:len(x)], psdx_dbm[0:len(x)])[0]
        #print target_range_idx
        peaks = target_range_idx * Fs/len(x)/8
        self.range_fft_peaks_handle = self.range_fft_subplot.scatter(peaks, psdx_dbm[target_range_idx], color='red')
        #self.range_fft_subplot_handle.set_xdata(x)
        #self.range_fft_subplot_handle.set_ydata(np.log10(range_fft[0:len(x)]))
        #self.range_fft_subplot.scatter(target_range_idx, np.log10(range_fft[target_range_idx]))
        #self.range_fft_peaks_handle.set_xdata(target_range_idx)
        #self.range_fft_peaks_handle.set_ydata(np.log10(range_fft[target_range_idx]))
        self.figure.canvas.draw()


class Radar_Tx_Signal_Plot(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BACKGROUND_COLOR)
        self.figure = Figure()
        self.canvas = FigureCanvas(self, 1, self.figure)
        #self.figure.set_title("Radar Target Plot")
        self.tx_signal_subplot = self.figure.add_subplot(111)
        self.tx_signal_subplot.set_title('Tx Signal')
        self.toolbar = NavigationToolbar(self.canvas)
        self.toolbar.Realize()

        #this seems hacky but it is the only way to start the prot
        self.tx_signal_subplot_handle = None

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.canvas, 1,  wx.EXPAND)
        self.sizer.Add(self.toolbar, 0, wx.HORIZONTAL | wx.EXPAND)
        self.SetSizer(self.sizer)
        pub.subscribe(self.update_view, "update_radar_table")

    def update_view(self, msg):
        radar_msg = Radar_Message(msg)
        if radar_msg:
            tx_signal = radar_msg.tx_waveform
            time_series = radar_msg.time_series
            self.update_tx_signal_plot(time_series, tx_signal)

    def update_tx_signal_plot(self, time_series, tx_signal):
        x = range(len(tx_signal))
        if self.tx_signal_subplot_handle == None:
            self.tx_signal_subplot_handle = self.tx_signal_subplot.plot(x,tx_signal)[0]

        self.tx_signal_subplot_handle.set_xdata(x)
        self.tx_signal_subplot_handle.set_ydata(tx_signal)
        self.figure.canvas.draw()


class Radar_Rx_Signal_Plot(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BACKGROUND_COLOR)
        self.figure = Figure()
        self.canvas = FigureCanvas(self, 1, self.figure)
        #self.figure.set_title("Radar Target Plot")
        self.rx_signal_subplot = self.figure.add_subplot(111)
        #self.rx_signal_subplot.autoscale(tight=True)
        self.rx_signal_subplot.set_title('Rx Dechirped Signal')

        self.toolbar = NavigationToolbar(self.canvas)
        self.toolbar.Realize()

        #this seems hacky but it is the only way to start the prot
        self.rx_signal_subplot_handle = None

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.canvas, 1,  wx.EXPAND)
        self.sizer.Add(self.toolbar, 0, wx.LEFT | wx.EXPAND)
        self.SetSizer(self.sizer)
        pub.subscribe(self.update_view, "update_radar_table")

    def update_view(self, msg):
        radar_msg = Radar_Message(msg)
        if radar_msg:
            rx_signal = radar_msg.rx_signal
            self.update_rx_signal_plot(rx_signal)

    def update_rx_signal_plot(self, rx_signal):
        #voltage_z0 = 50.0 * (1e3) ** 2
        #rx_signal = voltage_z0 * rx_signal
        x = range(len(rx_signal))
        if self.rx_signal_subplot_handle == None:
            self.rx_signal_subplot_handle = self.rx_signal_subplot.plot(x,rx_signal)[0]
        #signal_max = max(rx_signal.real)
        self.rx_signal_subplot.set_ylim([-250,250])
        self.rx_signal_subplot_handle.set_xdata(x)
        self.rx_signal_subplot_handle.set_ydata(rx_signal)
        self.figure.canvas.draw()


class Radar_Polar_Plot(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BACKGROUND_COLOR)
        self.figure = Figure()
        self.canvas = FigureCanvas(self, 1, self.figure)
        #self.figure.set_title("Radar Target Plot")
        #self.target_long_range_subplot = self.figure.add_subplot(211, polar=True)
        self.target_long_range_subplot = self.figure.add_axes([0, 0.45, 1, 0.4], polar=True)
        self.target_long_range_subplot.set_title('Radar Target Plot', pad=16)
        self.target_long_range_subplot.set_thetamin(-10)
        self.target_long_range_subplot.set_thetamax(10)
        self.target_long_range_subplot.set_rorigin(-40)
        self.target_long_range_subplot.set_ylim(40, 150)
        #self.target_long_range_subplot.set_
        self.target_long_range_subplot.set_theta_zero_location('N')

        self.target_mid_range_subplot = self.figure.add_axes([0, 0, 1, 0.4], polar=True)
        #self.target_mid_range_subplot = self.figure.add_subplot(212, polar=True)
        self.target_mid_range_subplot.set_thetamin(-45)
        self.target_mid_range_subplot.set_thetamax(45)
        self.target_mid_range_subplot.set_ylim(0, 60)
        self.target_mid_range_subplot.set_theta_zero_location('N')


        self.toolbar = NavigationToolbar(self.canvas)
        self.toolbar.Realize()
        self.targets_bounding_box = None

        pub.subscribe(self.update_view, 'update_radar_table')
        pub.subscribe(self.update_bounding, 'update_bounding_box')

        N = 20
        r = 150 * np.random.rand(N)
        theta = 2 * np.pi * np.random.rand(N)
        #area = .01*r**2
        #colors = theta

        #self.target_polar_handle = self.target_long_range_subplot.scatter(theta, r, c=colors, s=area, cmap='hsv', alpha =0.75)
        self.target_polar_handle = self.target_long_range_subplot.scatter(theta, r, marker='v', cmap='hsv', alpha =0.75)

        self.target_mid_range_subplot.scatter(theta, r, c='r', marker='s', cmap='hsv', alpha =0.75)

        self.string_time_radar = wx.StaticText(self, label="", style=wx.ELLIPSIZE_END)
        self.string_time_bb = wx.StaticText(self, label="", style=wx.ELLIPSIZE_END)

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.string_time_radar, 0, wx.HORIZONTAL | wx.EXPAND)
        self.sizer.Add(self.string_time_bb, 0, wx.HORIZONTAL | wx.EXPAND)
        self.sizer.Add(self.canvas, 1, wx.ALL | wx.EXPAND)
        self.sizer.Add(self.toolbar, 0, wx.HORIZONTAL | wx.EXPAND)
        self.SetSizer(self.sizer)
    
    def update_bounding(self, msg):
        if msg:
            self.targets_bounding_box = Bounding_Box_Message(msg)
            if self.targets_bounding_box != None:
                self.string_time_bb.SetLabelText('BB    GAMETIME: {0: .2f} \tTIMESTAMP: {1}'.
                                          format(self.targets_bounding_box.game_time,
                                                 self.targets_bounding_box.time_stamp))
            #self.update_plot(self.targets)
        else:
            print("empty bounding target list")

    def update_view(self, msg):
        if msg:
            self.targets = Radar_Message(msg)
            self.update_plot(self.targets)
            self.string_time_radar.SetLabelText('Radar GAMETIME: {0: .2f} \tTIMESTAMP: {1}'.
                                          format(self.targets.game_time, self.targets.time_stamp))
        else:
            print("empty target list")

    def update_plot(self, targets):
        if len(targets.ranges) > 0:
            self.set_data(targets)
        #self.Layout()
        self.Refresh()
    
    def set_data(self, targets):
        r = targets.ranges
        theta = -np.radians(targets.aoa_list)
        rcs = targets.rcs_list
        if self.targets_bounding_box:
            bounding_box_distances = self.targets_bounding_box.radar_distances
            bounding_box_angles = -np.radians(self.targets_bounding_box.radar_angles)
        #speed = targets.velocities/100
        #there seems to be a bug in the new polar plot library, set_offsets is not working
        #so we have to do all the following on every frame
        self.target_long_range_subplot.cla()
        #self.target_long_range_subplot.set_title('Radar Target Plot')
        self.target_long_range_subplot.set_thetamin(-10)
        self.target_long_range_subplot.set_thetamax(10)
        self.target_long_range_subplot.set_ylim(40, 150)
        self.target_long_range_subplot.set_theta_zero_location('N')
        self.target_long_range_subplot.set_rorigin(-40)
        self.target_long_range_subplot.scatter(theta, r, c='r', marker='s', cmap='hsv', alpha =0.75)
        self.target_long_range_subplot.scatter(bounding_box_angles, bounding_box_distances, s=100, facecolors='none', edgecolors='b', cmap='hsv', alpha =0.75)

        self.target_mid_range_subplot.cla()
        self.target_mid_range_subplot.set_thetamin(-45)
        self.target_mid_range_subplot.set_thetamax(45)
        self.target_mid_range_subplot.set_ylim(0, 60)
        self.target_mid_range_subplot.set_theta_zero_location('N')
        self.target_mid_range_subplot.scatter(theta, r, c='r', marker='s', cmap='hsv', alpha =0.75)
        self.target_mid_range_subplot.scatter(bounding_box_angles, bounding_box_distances, s=100, facecolors='none', edgecolors='b', cmap='hsv', alpha =0.75)
        #self.target_polar_handle.set_offsets([theta,r])
        self.figure.canvas.draw()


class Radar_Target_Table(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BACKGROUND_COLOR)
        self.figure = Figure()
        self.canvas = FigureCanvas(self, 1, self.figure)
        self.target_table_subplot = self.figure.add_subplot(111)
        self.target_table_subplot.set_title('Target Table')
        self.target_table_subplot.axis('tight')
        self.target_table_subplot.axis('off')
        self.target_table_subplot.grid(visible=True)
        self.toolbar = NavigationToolbar(self.canvas)
        self.toolbar.Realize()
        self.target_table_handle = None
        self.old_size = 0
        self.max_number_of_targets = 24

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.canvas, 1,  wx.EXPAND)
        self.sizer.Add(self.toolbar, 0, wx.LEFT | wx.EXPAND)
        self.SetSizer(self.sizer)

        pub.subscribe(self.update_view, 'update_radar_table')
    
    def update_view(self, msg):
        if msg:
            self.targets = Radar_Message(msg)
            self.update_plot(self.targets)
        else:
            print("empty target list")

    def update_plot(self,targets):
        if self.target_table_handle == None and len(targets.ranges) > 0:
            self.target_table_handle = self.setup_radar_plots(targets)
        if len(targets.ranges) > 0:
            self.set_data(self.targets)
        self.figure.canvas.draw()
        #self.Layout()
        self.Refresh()

    def setup_radar_plots(self, targets={}):
        targets_cells = np.array(self.max_number_of_targets * [6 * [0]])
        targets_handle = self.target_table_subplot.table(cellText=targets_cells[0:self.max_number_of_targets],
                                          colLabels=("Target", "Range", "Speed", "AoA", "RCS", "Power Level (dB)"),
                                          loc='center')
        targets_handle.auto_set_font_size(False)
        targets_handle.set_fontsize(5.5)
        for i in range(0,6):
            for j in range(self.max_number_of_targets + 1, self.max_number_of_targets + 1):
                    targets_handle._cells[(j,i)]._text.set_text('')
                    targets_handle._cells[(j,i)].set_linewidth(0)

        self.old_size = self.max_number_of_targets

        return targets_handle

    def set_data(self,targets):
        target_cells = np.array(self.max_number_of_targets * [6 * [0]])
        if len(targets.ranges):
            target_cells[0:self.max_number_of_targets,    0] = range(0, self.max_number_of_targets)
            target_cells[0:len(targets.ranges),     1] = targets.ranges
            target_cells[0:len(targets.velocities), 2] = targets.velocities
            target_cells[0:len(targets.aoa_list),   3] = targets.aoa_list
            target_cells[0:len(targets.rcs_list),   4] = targets.rcs_list
            target_cells[0:len(targets.power_list), 5] = targets.power_list

        '''for i in range(0, 6):
            for j in range(1, self.old_size + 1): #to erase previous display
                try:
                    self.target_table_handle._cells[(j,i)]._text.set_text('')
                    self.target_table_handle._cells[(j,i)].set_linewidth(0)
                except:
                    pass'''

        for i in range(0,6):
            for j in range(1, self.max_number_of_targets + 1): #to refresh with new display
                try:
                    self.target_table_handle._cells[(j, i)]._text.set_text(target_cells[j - 1, i])
                    self.target_table_handle._cells[(j, i)].set_linewidth(1)
                    if j > len(targets.ranges + 1) and i > 0:
                       self.target_table_handle._cells[(j, i)]._text.set_text('---')
                except:
                    pass

        self.old_size = self.max_number_of_targets


class RoadMap_View(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BACKGROUND_COLOR)
        self.figure = Figure()
        self.canvas = FigureCanvas(self, 1, self.figure)
        self.map_subplot = self.figure.add_subplot(111)
        self.toolbar = NavigationToolbar(self.canvas)
        self.toolbar.Realize()
        
        #this seems hacky but it is the only way to start the prot
        self.map_subplot_handle = None
        self.map_subplot.set_title("Ground Truth Map View")

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.canvas, 1,  wx.EXPAND)
        self.sizer.Add(self.toolbar, 0, wx.LEFT | wx.EXPAND)
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


class GPS_View(BufferedWindow):
    def __init__(self, parent, *args, **kwargs):
        self.string_lat = ""
        self.string_lng = ""
        self.string_time = ""
        self.font = wx.Font(9, wx.FONTFAMILY_SWISS, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        BufferedWindow.__init__(self, parent, *args, **kwargs)
        self.SetMinSize(wx.Size(100, 16*3))
        self.SetBackgroundColour(INNER_PANEL_COLOR)
        self.UpdateDrawing()

        # self.string_lat = wx.StaticText(self, label="")
        # self.string_lng = wx.StaticText(self, label="")
        # self.string_time = wx.StaticText(self, label="", style=wx.ELLIPSIZE_END)
        #
        # self.sizer = wx.BoxSizer(wx.VERTICAL)
        # self.sizer.Add(self.string_lat, 1, wx.HORIZONTAL | wx.EXPAND | wx.ALIGN_CENTER_VERTICAL)
        # self.sizer.Add(self.string_lng, 1, wx.HORIZONTAL | wx.EXPAND | wx.ALIGN_CENTER_VERTICAL)
        # self.sizer.Add(self.string_time, 1, wx.HORIZONTAL | wx.EXPAND | wx.ALIGN_CENTER_VERTICAL)
        # self.SetSizerAndFit(self.sizer)

        pub.subscribe(self.update_view, "update_gps")

    def update_view(self, msg):
        #print("GPS update view:", msg)
        gps_msg = GPS_Message(msg)
        self.string_lat = 'LAT: {:0.8f}'.format(gps_msg.lat)
        self.string_lng = 'LNG: {:0.8f}'.format(gps_msg.lng)
        self.string_time = 'GAMETIME: {0: .2f} \tTIMESTAMP: {1}'.format(gps_msg.game_time, gps_msg.time_stamp)
        # self.string_lat.SetLabelText('LAT: {0}'.format(gps_msg.lat))
        # self.string_lng.SetLabelText('LNG: {0}'.format(gps_msg.lng))
        # self.string_time.SetLabelText('GAMETIME: {0: .2f} \tTIMESTAMP: {1}'.format(gps_msg.game_time, gps_msg.time_stamp))
        self.UpdateDrawing()

    def Draw(self, dc):
        dc.SetBackground(wx.Brush(INNER_PANEL_COLOR))
        dc.Clear()

        dc.SetFont(self.font)

        left = 2
        y = 0
        size = dc.GetTextExtent("X")
        height = size[1]

        dc.DrawText(self.string_lat, left, y)
        y += height*2

        dc.DrawText(self.string_lng, left, y)
        y += height*2

        dc.DrawText(self.string_time, left, y)
        y += height*2

class IMU_View(BufferedWindow):
    def __init__(self, parent, *args, **kwargs):

        self.font = wx.Font(9, wx.FONTFAMILY_SWISS, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_NORMAL)
        self.string_accel_x = ""
        self.string_accel_y = ""
        self.string_accel_z = ""
        self.string_ang_rate_x = ""
        self.string_ang_rate_y = ""
        self.string_ang_rate_z = ""
        self.string_timer = ""
        BufferedWindow.__init__(self, parent, *args, **kwargs)
        self.SetMinSize(wx.Size(100, 16*7))
        self.SetBackgroundColour(INNER_PANEL_COLOR)
        self.UpdateDrawing()

        # self.string_accel_x = wx.StaticText(self, label="")
        # self.string_accel_y = wx.StaticText(self, label="")
        # self.string_accel_z = wx.StaticText(self, label="")
        # self.string_ang_rate_x = wx.StaticText(self, label="")
        # self.string_ang_rate_y = wx.StaticText(self, label="")
        # self.string_ang_rate_z = wx.StaticText(self, label="")
        # self.string_timer = wx.StaticText(self, label="", style=wx.ELLIPSIZE_END)
        #
        # self.sizer = wx.BoxSizer(wx.VERTICAL)
        # self.sizer.Add(self.string_accel_x, 1,  wx.EXPAND)
        # self.sizer.Add(self.string_accel_y, 1,  wx.EXPAND)
        # self.sizer.Add(self.string_accel_z, 1,  wx.EXPAND)
        # self.sizer.Add(self.string_ang_rate_x, 1,  wx.EXPAND)
        # self.sizer.Add(self.string_ang_rate_y, 1,  wx.EXPAND)
        # self.sizer.Add(self.string_ang_rate_z, 1,  wx.EXPAND)
        # self.sizer.Add(self.string_timer, 1, wx.HORIZONTAL | wx.EXPAND)
        # self.SetSizerAndFit(self.sizer)

        pub.subscribe(self.update_view, "update_imu")

    def update_view(self, msg):
        imu_msg = IMU_Message(msg)
        self.string_accel_x = 'ACCEL_X: {:0.8f}'.format(imu_msg.string_accel_x)
        self.string_accel_y = 'ACCEL_Y: {:0.8f}'.format(imu_msg.string_accel_y)
        self.string_accel_z = 'ACCEL_Z: {:0.8f}'.format(imu_msg.string_accel_z)
        self.string_ang_rate_x = 'ANG RATE X: {:0.8f}'.format(imu_msg.string_ang_rate_x)
        self.string_ang_rate_y = 'ANG RATE Y: {:0.8f}'.format(imu_msg.string_ang_rate_y)
        self.string_ang_rate_z = 'ANG RATE X: {:0.8f}'.format(imu_msg.string_ang_rate_z)
        self.string_timer = 'GAMETIME: {0: .2f} \tTIMESTAMP: {1}'.format(imu_msg.game_time, imu_msg.time_stamp)
        # self.string_accel_x.SetLabelText('ACCEL_X: {0}'.format(imu_msg.string_accel_x))
        # self.string_accel_y.SetLabelText('ACCEL_Y: {0}'.format(imu_msg.string_accel_y))
        # self.string_accel_z.SetLabelText('ACCEL_Z: {0}'.format(imu_msg.string_accel_z))
        # self.string_ang_rate_x.SetLabelText('ANG RATE X: {0}'.format(imu_msg.string_ang_rate_x))
        # self.string_ang_rate_y.SetLabelText('ANG RATE Y: {0}'.format(imu_msg.string_ang_rate_y))
        # self.string_ang_rate_z.SetLabelText('ANG RATE X: {0}'.format(imu_msg.string_ang_rate_z))
        # self.string_timer.SetLabelText('GAMETIME: {0: .2f} \tTIMESTAMP: {1}'.format(imu_msg.game_time, imu_msg.time_stamp))
        self.UpdateDrawing()

    def Draw(self, dc):
        dc.SetBackground(wx.Brush(INNER_PANEL_COLOR))
        dc.Clear()

        dc.SetFont(self.font)

        left = 2
        y = 0
        size = dc.GetTextExtent("X")
        height = size[1]

        dc.DrawText(self.string_accel_x, left, y)
        y += height

        dc.DrawText(self.string_accel_y, left, y)
        y += height

        dc.DrawText(self.string_accel_z, left, y)
        y += height

        dc.DrawText(self.string_ang_rate_x, left, y)
        y += height

        dc.DrawText(self.string_ang_rate_y, left, y)
        y += height

        dc.DrawText(self.string_ang_rate_z, left, y)
        y += height

        dc.DrawText(self.string_timer, left, y)
        y += height


class Wheel_RPM_View(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(INNER_PANEL_COLOR)

        self.string_rpm_lf = wx.StaticText(self, label="")
        self.string_rpm_rf = wx.StaticText(self, label="")
        self.string_rpm_lr = wx.StaticText(self, label="")
        self.string_rpm_rr = wx.StaticText(self, label="")

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.string_rpm_lf, 1, wx.EXPAND)
        self.sizer.Add(self.string_rpm_rf, 1, wx.EXPAND)
        self.sizer.Add(self.string_rpm_lr, 1, wx.EXPAND)
        self.sizer.Add(self.string_rpm_rr, 1, wx.EXPAND)
        self.SetSizer(self.sizer)


class Camera_View(BufferedWindow):
    def __init__(self, parent, *args, **kwargs):
        self.png = wx.Image(filepath, wx.BITMAP_TYPE_ANY)
        self.current_bitmap = self.bitmap_from_image(self.png)
        self.current_size = wx.Size(self.current_bitmap.GetWidth(), self.current_bitmap.GetHeight())
        BufferedWindow.__init__(self, parent, *args, **kwargs)
        self.SetMinSize(self.current_size)
        self.impil = None

        pub.subscribe(self.update_view, "update_camera")
        self.UpdateDrawing()

    def create_image(self, width, height):
        if sys.version_info[0] == 2:
            return wx.EmptyImage(width, height)
        return wx.Image(width, height)

    def bitmap_from_image(self, image):
        if sys.version_info[0] == 2:
            return wx.BitmapFromImage(image)
        return wx.Bitmap(image)

    def image_from_bitmap(self, bitmap):
        if sys.version_info[0] == 2:
            return wx.ImageFromBitmap(bitmap)
        return bitmap.ConvertToImage()

    def update_view(self, msg):
        camera_msg = Camera_Message(msg)
        self.current_bitmap = self.to_bmp(camera_msg.np_image)
        size = wx.Size(self.current_bitmap.GetWidth(), self.current_bitmap.GetHeight())
        if size != self.current_size:
            self.current_size = wx.Size(self.current_bitmap.GetWidth(), self.current_bitmap.GetHeight())
            self.SetMinSize(self.current_size)
            #print("update camera view(%s,%s)" % (self.current_bitmap.GetWidth(), self.current_bitmap.GetHeight()))

        rect = wx.Rect((self.ClientSize.x - self.current_size.x) / 2, (self.ClientSize.y - self.current_size.y) / 2,
            self.current_size.x, self.current_size.y)
        self.UpdateDrawing(rect)

    def to_bmp(self, np_image):
        imcv = cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)
        self.impil = Image.fromarray(imcv)
        imwx = self.create_image(self.impil.size[0], self.impil.size[1])
        imwx.SetData(self.impil.convert('RGB').tobytes()) 
        bitmap = self.bitmap_from_image(imwx)
        return self.scale_bitmap(bitmap)

    def scale_bitmap(self, bitmap):
        frame_size = self.ClientSize
        bitmap_size = bitmap.GetSize()
        if frame_size.x > 0 and frame_size.y > 0 and \
                (frame_size.x < bitmap_size.x or frame_size.y < bitmap_size.y):
            aspect = float(bitmap.GetWidth()) / float(bitmap.GetHeight())
            image = self.image_from_bitmap(bitmap)
            frame_h = frame_size.y
            frame_w = frame_size.y * aspect
            image = image.Scale(frame_w, frame_h, wx.IMAGE_QUALITY_HIGH)
            scaled_bitmap = self.bitmap_from_image(image)
        else:
            scaled_bitmap = bitmap
        return scaled_bitmap

    def Draw(self, dc):
        dc.SetBackground(wx.Brush("White"))
        dc.Clear()

        if self.current_bitmap is not None:
            Size = self.ClientSize
            dc.DrawBitmap(self.current_bitmap,
                          max(0, (Size.x - self.current_size.x) / 2),
                          max(0, (Size.y - self.current_size.y) / 2))


class Radar_Panel(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent,*args, **kwargs)
        self.radar_tx_signal = Radar_Tx_Signal_Plot(self)
        self.radar_target_table = Radar_Target_Table(self)
        self.radar_polar_plot = Radar_Polar_Plot(self)
        self.rx_signal_details = GraphRow(self)

        self.main_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.main_sizer.Add(self.radar_tx_signal, 1, wx.EXPAND | wx.ALL, border=2)
        self.main_sizer.Add(self.rx_signal_details, 1, wx.EXPAND | wx.ALL, border=2)
        self.main_sizer.Add(self.radar_polar_plot, 1, wx.EXPAND | wx.ALL, border=2)
        self.main_sizer.Add(self.radar_target_table, 1, wx.EXPAND | wx.ALL, border=2)
        self.SetSizerAndFit(self.main_sizer)

        self.rx_signal_plot = Radar_Rx_Signal_Plot(self.rx_signal_details)
        self.range_fft_plot = Radar_FFT_Plot(self.rx_signal_details)

        self.signal_details_sizer = wx.BoxSizer(wx.VERTICAL)
        self.signal_details_sizer.Add(self.rx_signal_plot, 1, wx.EXPAND | wx.ALL, border=2)
        self.signal_details_sizer.Add(self.range_fft_plot, 1, wx.EXPAND | wx.ALL, border=2)
        self.rx_signal_details.SetSizer(self.signal_details_sizer)

        self.Layout()
        self.Refresh()


class Overview_Panel(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent,*args, **kwargs)
        #set up frame panels

        self.graph_row_panel = GraphRow(self)
        self.text_row_panel = TextRow(self)
        self.camera_row_panel = CameraRow(self)

        #set up sizers for frame panels
        self.main_sizer = wx.BoxSizer(wx.VERTICAL)
        self.main_sizer.Add(self.graph_row_panel, 1, wx.ALL | wx.EXPAND, border=2)
        self.main_sizer.Add(self.text_row_panel, 0, wx.HORIZONTAL | wx.EXPAND, border=2)
        self.main_sizer.Add(self.camera_row_panel, 1, wx.ALL | wx.EXPAND, border=2)
        self.SetSizer(self.main_sizer)

        #add figures to graph row
        self.roadmap_view = RoadMap_View(self.graph_row_panel)
        self.radar_polar_plot = Radar_Polar_Plot(self.graph_row_panel)
        self.bounding_box_plot = Bounding_Polar_Plot(self.graph_row_panel)

        self.graph_row_panel_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.graph_row_panel_sizer.Add(self.roadmap_view, 1, wx.EXPAND | wx.ALL, border=2)
        self.graph_row_panel_sizer.Add(self.radar_polar_plot, 1, wx.EXPAND | wx.ALL, border=2)
        self.graph_row_panel_sizer.Add(self.bounding_box_plot, 1, wx.EXPAND | wx.ALL, border=2)
        self.graph_row_panel.SetSizerAndFit(self.graph_row_panel_sizer)
 
        #add panels to center row and size them
        self.gps_view = GPS_View(self.text_row_panel)
        self.imu_view = IMU_View(self.text_row_panel)
        self.wheel_rpm_view = Wheel_RPM_View(self.text_row_panel)
        self.text_row_panel_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.text_row_panel_sizer.Add(self.gps_view, 1, wx.EXPAND|wx.ALL, border=2)
        self.text_row_panel_sizer.Add(self.imu_view, 1, wx.EXPAND|wx.ALL, border=2)
        self.text_row_panel_sizer.Add(self.wheel_rpm_view, 1, wx.EXPAND|wx.ALL, border=2)
        self.text_row_panel.SetSizerAndFit(self.text_row_panel_sizer)

        #add camera to bottom row and size it
        self.camera_view = Camera_View(self.camera_row_panel)
        self.camera_row_panel_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.camera_row_panel_sizer.Add(self.camera_view, 1, wx.EXPAND | wx.ALL, border=2)
        self.camera_row_panel.SetSizerAndFit(self.camera_row_panel_sizer)

        self.Bind(wx.EVT_SIZE, self.OnSize)

        self.Layout()
        self.Refresh()

        #self.Destroy()

    def OnSize(self, evt):
        GraphHeaders = 68
        Size = self.ClientSize
        TextSize = self.text_row_panel.GetBestSize()
        h = (Size.y - TextSize.y - GraphHeaders) / 2

        #print("OnSize(%s,%s) Text:(%s,%s) h:%s" % (Size.x, Size.y, TextSize.x, TextSize.y, h))
        min_h = min(Size.x / 3, h)

        self.roadmap_view.SetMinSize(wx.Size(min_h, min_h))
        self.radar_polar_plot.SetMinSize(wx.Size(min_h, min_h))
        self.bounding_box_plot.SetMinSize(wx.Size(min_h, min_h))

        self.graph_row_panel.SetMinSize(wx.Size(Size.x, h + GraphHeaders))
        self.camera_row_panel.SetMinSize(wx.Size(Size.x, h))

        self.graph_row_panel.Layout()
        self.Layout()

class MainFrame(wx.Frame):
    def __init__(self):
        width = int(wx.SystemSettings.GetMetric(wx.SYS_SCREEN_X) * .9)
        height = int(wx.SystemSettings.GetMetric(wx.SYS_SCREEN_Y) * .9)
        wx.Frame.__init__(self, None, size=(width,height), title = "monoDrive Visualizer")
        pub.subscribe(self.shutdown, "SHUTDOWN")
        # Here we create a panel and a notebook on the panel
        p = wx.Panel(self)
        nb = wx.Notebook(p)

        # create the page windows as children of the notebook
        overview = Overview_Panel(nb)
        radar = Radar_Panel(nb)
        camera = Camera_View(nb)

        # add the pages to the notebook with the label to show on the tab
        nb.AddPage(overview, "All Sensors")
        nb.AddPage(radar, "Radar")
        nb.AddPage(camera, "Camera")

        # finally, put the notebook in a sizer for the panel to manage
        # the layout
        sizer = wx.BoxSizer()
        sizer.Add(nb, 1, wx.EXPAND)
        p.SetSizer(sizer)

    def shutdown(self, msg):
        print("Frame shutting down {0}".format(msg))
        self.Close()


#Ctrl-Alt-I, or Cmd-Alt-I on Mac for inspection app for viewing layout
class MonoDriveGUIApp(wx.App, wx.lib.mixins.inspection.InspectionMixin):
#class MonoDriveGUIApp(wx.App):
    def OnInit(self):
        self.Init()  # initialize the inspection tool
        self.MainWindow = MainFrame()
        self.MainWindow.Show(True)
        self.SetTopWindow(self.MainWindow)
        #wx.lib.inspection.InspectionTool().Show()
        return True

    def Close(self):
        try:
            wx.CallAfter(self.MainWindow.Close)
            time.sleep(0.2)
        except: pass
        wx.CallAfter(self.Destroy)
        time.sleep(0.2)


class SensorPoll(Thread):
    """Thread to pull data from sensor q's and publish to views"""
    def __init__(self, sensors, fps, map, clock_mode, sync_event):
        super(SensorPoll,self).__init__()
        #self.vehicle = vehicle
        #TODO need to fix the map getting here
        #self.road_map = vehicle.get_road_map()
        self.road_map = map
        #self.sensors = vehicle.get_sensors()
        self.sensors = sensors
        self.stop_event = multiprocessing.Event()
        self.sync_event = sync_event
        self.clock_mode = clock_mode
        self.fps = fps
        self.update_gui_rate = 1.0 / float(fps)
        self.start()

    def update_sensor_widget(self, sensor):

        messages = sensor.get_display_messages(block=True, timeout=0.0)
        found_data = len(messages) > 0
        if found_data:
            #print("{0} found {1} messages".format(sensor.name, len(messages)))
            message = messages.pop() #pop last message aka. most recent
            if "IMU" in sensor.name:
                wx.CallAfter(pub.sendMessage, "update_imu", msg=message)
            elif "GPS" in sensor.name:
                wx.CallAfter(pub.sendMessage, "update_gps", msg=message)
            elif "Camera" in sensor.name:
                message['width'] = sensor.width
                message['height'] = sensor.height
                wx.CallAfter(pub.sendMessage, "update_camera", msg=message)
            elif "Bounding" in sensor.name:
                wx.CallAfter(pub.sendMessage, "update_bounding_box", msg=message)
            elif "Radar" in sensor.name:
                wx.CallAfter(pub.sendMessage, "update_radar_table", msg=message)
            elif "Lidar" in sensor.name:
                sensor.forward_data(message)
        return found_data

    def check_client_mode_update(self, sensor):
        should_stop = False
        wait_time = 0
        updated = False
        while not updated and not should_stop and wait_time < 2.0:
            # if wait_time == 0:
            #     print("Client_Mode - sensor data not available: " + sensor.name)
            should_stop = self.stop_event.wait(self.update_gui_rate)
            updated = self.update_sensor_widget(sensor)
            wait_time += self.update_gui_rate

        if not updated:
            print("Client_Mode - sensor data not found(%s): %s" % (should_stop, sensor.name))
        return should_stop

    def update_gui(self, sensor):
        updated = self.update_sensor_widget(sensor)
        if self.clock_mode == ClockMode_ClientStep:
            self.sync_event.set()
            if not updated and self.check_client_mode_update(sensor):
                return False
        return True

    #this thread will run while application is running
    def run(self):
        while not self.stop_event.wait(self.update_gui_rate):
            # print("GUI THREAD RUNNING")
            for sensor in self.sensors:
                #print("GUI THREAD updating: ", sensor.name)
                if not self.update_gui(sensor):
                    break

            #     print("GUI THREAD STOPPED")
            if self.road_map:
                wx.CallAfter(pub.sendMessage, "update_roadmap", msg=self.road_map)
        #print("GUI THREAD NOT RUNNING")

    def stop(self, timeout=2):
        self.stop_event.set()
        self.join(timeout=timeout)

class GUISensor(object):
    def __init__(self, sensor, **kwargs):
        self.name = sensor.name
        self.queue = sensor.q_display
        if 'Camera' in sensor.name:
            self.width = sensor.width
            self.height = sensor.height

    def get_display_messages(self, block=True, timeout=None):

        messages = []
        try:
            msg = self.queue.get(block=block, timeout=timeout)
            messages.append(msg)
            while self.queue.qsize():
                msg = self.queue.get(block=True, timeout=0)
                messages.append(msg)
                # If `False`, the program is not blocked. `Queue.Empty` is thrown if
                # the queue is empty
        except Exception as e:
            #print("{0} Display Q_EMPTY".format(self.name))
            # if len(messages) == 0:
            #     messages.append("NO_DATA")
            pass
        return messages


class LidarGUISensor(GUISensor):
    def __init__(self, sensor, **kwargs):
        super(LidarGUISensor, self).__init__(sensor, **kwargs)
        self.connect()

    def forward_data(self, data):
        if self.veloview_socket is None:
            return

        if isinstance(data, list):
            for datum in data:
                self.veloview_socket.sendto(datum, (VELOVIEW_IP, VELOVIEW_PORT))
        else:
            self.veloview_socket.sendto(data, (VELOVIEW_IP, VELOVIEW_PORT))

    def connect(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.veloview_socket = s
            return s
        except Exception as e:
            print('Can send to {0}'.format(str((VELOVIEW_PORT, VELOVIEW_PORT))))
            print("Error {0}".format(e))
            self.veloview_socket = None
            return None


class GUI(object):
    def __init__(self, simulator, **kwargs):
        super(GUI, self).__init__(**kwargs)
        self.daemon = True
        self.name = "GUI"
        self.simulator_event = simulator.restart_event
        self.vehicle_sync_event = simulator.ego_vehicle.vehicle_drive
        self.vehicle_clock_mode = simulator.ego_vehicle.vehicle_config.clock_mode
        #self.simulator = simulator
        #self.vehicle = None
        #self.vehicle = simulator.ego_vehicle

        self.sensors = []
        for sensor in simulator.ego_vehicle.sensors:
            if "Lidar" in sensor.name:
                self.sensors.append(LidarGUISensor(sensor))
            else:
                self.sensors.append(GUISensor(sensor))

        self.app = None
        self.fps = None
        self.settings = simulator.configuration.gui_settings
        self.init_settings(self.settings)
        self.map = simulator.map_data

        self.stop_event = multiprocessing.Event()
        self.process = None
        self.sensor_polling = None
        self.start()
    
    def init_settings(self, settings):
        default_update_rate = 1.0
        if settings:
            self.settings = settings
            self.fps = self.settings['fps']
        else:
            self.fps = default_update_rate

    def start(self):
        self.process = multiprocessing.Process(target=self.run)
        self.process.name = self.name
        self.process.start()

    def stop(self, timeout=2):
        if self.process.is_alive():
            self.stop_event.set()
        self.join(timeout=timeout)

    def join(self, timeout=2):
        try:
            self.process.join(timeout=timeout)
        except Exception as e:
            print("could not join process {0} -> {1}".format(self.name, e))

    def run(self):

        monitor = Thread(target=self.monitor_process_state)
        monitor.start()

        self.app = MonoDriveGUIApp()

        #prctl.set_proctitle("mono{0}".format(self.name))
        #start sensor polling
        self.sensor_polling = SensorPoll(self.sensors, self.fps, self.map, self.vehicle_clock_mode, self.vehicle_sync_event)
        while not self.stop_event.is_set():
            self.app.MainLoop()
            self.simulator_event.set()
            time.sleep(0.5)

        monitor.join(timeout=2)

        print("GUI main DONE")

    #this will simply wait for sensor to stop running - and kill the app
    def monitor_process_state(self):
        self.stop_event.wait()
        self.sensor_polling.stop()
        self.app.Close()
        #print("exiting monitor process")


if __name__ == '__main__':
    #vehicle = "vehicle"
    #gui = GUI(vehicle)
    print("running MonoDriveGUIApp")
    app = MonoDriveGUIApp()
    if app:
        app.MainLoop()
    else:
        print("MonoDriveGUI app not running")