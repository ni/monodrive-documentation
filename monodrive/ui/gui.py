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

import cPickle as pickle
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
import prctl

import time

from message import IMU_Message
from message import GPS_Message
from message import Camera_Message
from message import MapData

BACKGROUND_COLOR = '#eaf7ff'
INNER_PANEL_COLOR = '#f0f0f0'
BLACK = '#000000'
WHITE = '#ffffff'


class TextRow(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BLACK)

class GraphRow(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BLACK)

class CameraRow(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BLACK)

class RoadMap_View(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(BACKGROUND_COLOR)
        #self.string_map = wx.StaticText(self, label="")
        #self.sizer = wx.BoxSizer(wx.VERTICAL)
        #self.sizer.Add(self.string_map, 1, wx.LEFT | wx.RIGHT | wx.EXPAND)
        #self.SetSizerAndFit(self.sizer)
        self.figure = Figure()
        self.canvas = FigureCanvas(self, 1, self.figure)
        self.map_subplot = self.figure.add_subplot(111)
        self.toolbar = NavigationToolbar(self.canvas)
        self.toolbar.Realize()
        
        #self.map_subplot.autoscale_view(True)
    
        #self.map_subplot_handle = self.map_subplot.plot(0, 0, marker='.', linestyle='None')[0]
        #this seems hacky but it is the only way to start the
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
        print("Roadmap update view")
        self.road_map = MapData(msg.data)
        if self.road_map:
            print("map: r:{0} l0:{1}".format(self.num_roads(), self.num_lanes(0)))

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
        #self.map_subplot_handle.set_xdata(x)
        #self.map_subplot_handle.set_ydata(y)
        #self.map_subplot.autoscale_view(True)
        #self.map_subplot.autoscale_view(True,True,True)
        #self.map_subplot.draw()
        #margin = 10
        #self.map_subplot_handle.axis((min(x) - margin, max(x) + margin, min(y) - margin, max(y) + margin))
        if self.map_subplot_handle == None:
            self.map_subplot_handle = self.map_subplot.plot(x, y, marker='.', linestyle='None')[0]
        self.map_subplot_handle.set_xdata(x)
        self.map_subplot_handle.set_ydata(y)
        self.Layout()
        self.Refresh()


class GPS_View(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(INNER_PANEL_COLOR)

        self.string_lat = wx.StaticText(self, label="")
        self.string_lng = wx.StaticText(self, label="")
        self.string_time = wx.StaticText(self, label="")

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.string_lat,1, wx.LEFT | wx.RIGHT | wx.EXPAND)
        self.sizer.Add(self.string_lng,1, wx.LEFT | wx.RIGHT | wx.EXPAND)
        self.sizer.Add(self.string_time, 1, wx.LEFT | wx.RIGHT | wx.EXPAND)
        self.SetSizerAndFit(self.sizer)

        pub.subscribe(self.update_view, "update_gps")

    def update_view(self, msg):    
        self.string_lat.SetLabelText('LAT: {0}'.format(msg['lat']))
        self.string_lng.SetLabelText('LNG: {0}'.format(msg['lng']))
        self.string_time.SetLabelText('TIMESTAMP: {0}'.format(msg['time_stamp']))


class IMU_View(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.SetBackgroundColour(INNER_PANEL_COLOR)

        self.string_accel_x = wx.StaticText(self, label="")
        self.string_accel_y = wx.StaticText(self, label="")
        self.string_accel_z = wx.StaticText(self, label="")
        self.string_ang_rate_x = wx.StaticText(self, label="")
        self.string_ang_rate_y = wx.StaticText(self, label="")
        self.string_ang_rate_z = wx.StaticText(self, label="")
        self.string_timer = wx.StaticText(self, label="")

        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.string_accel_x, 1,  wx.EXPAND)
        self.sizer.Add(self.string_accel_y, 1,  wx.EXPAND)
        self.sizer.Add(self.string_accel_z, 1,  wx.EXPAND)
        self.sizer.Add(self.string_ang_rate_x, 1,  wx.EXPAND)
        self.sizer.Add(self.string_ang_rate_y, 1,  wx.EXPAND)
        self.sizer.Add(self.string_ang_rate_z, 1,  wx.EXPAND)
        self.sizer.Add(self.string_timer, 1,  wx.EXPAND)
        self.SetSizerAndFit(self.sizer)

        pub.subscribe(self.update_view, "update_imu")

    def update_view(self, msg):
        imu_msg = IMU_Message(msg)
        self.string_accel_x.SetLabelText('ACCEL_X: {0}'.format(imu_msg.string_accel_x))
        self.string_accel_y.SetLabelText('ACCEL_Y: {0}'.format(imu_msg.string_accel_y))
        self.string_accel_z.SetLabelText('ACCEL_Z: {0}'.format(imu_msg.string_accel_z))
        self.string_ang_rate_x.SetLabelText('ANG RATE X: {0}'.format(imu_msg.string_ang_rate_x))
        self.string_ang_rate_y.SetLabelText('ANG RATE Y: {0}'.format(imu_msg.string_ang_rate_y))
        self.string_ang_rate_z.SetLabelText('ANG RATE X: {0}'.format(imu_msg.string_ang_rate_z))
        self.string_timer.SetLabelText('TIMESTAMP: {0}'.format(imu_msg.time_stamp))

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

class Camera_View(wx.Panel):
    def __init__(self, parent, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
        self.impil = None
        self.png = wx.Image(filepath, wx.BITMAP_TYPE_ANY)
        self.bmwx = wx.StaticBitmap(self, wx.ID_ANY, wx.BitmapFromImage(self.png),(0, 0))
        self.sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.sizer.Add(self.bmwx,1,wx.EXPAND | wx.TOP | wx.BOTTOM)
        self.SetSizer(self.sizer)
        #self.Bind(wx.EVT_SIZE, self.onResize)
        pub.subscribe(self.update_view, "update_camera")

    def update_view(self, msg):
        camera_msg = Camera_Message(msg)
        bitmap = self.to_bmp(camera_msg.np_image)
        self.bmwx.SetBitmap(bitmap)
        self.Refresh()
        self.Layout()

    def to_bmp(self, np_image):
        imcv = cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)
        self.impil = Image.fromarray(imcv)
        imwx = wx.EmptyImage(self.impil.size[0], self.impil.size[1])
        imwx.SetData(self.impil.convert('RGB').tobytes()) 
        bitmap = wx.BitmapFromImage(imwx)
        #scaled_bitmap = self.scale_bitmap(bitmap)
        return bitmap

    def scale_bitmap(self, bitmap):
        frame_size = self.GetBestSize()
        frame_h = (frame_size[0]-10)
        frame_w = (frame_size[1]-10)
        image = wx.ImageFromBitmap(bitmap)
        image = image.Scale(frame_w, frame_h, wx.IMAGE_QUALITY_HIGH)
        scaled_bitmap = wx.BitmapFromImage(image)
        return scaled_bitmap

    '''def onResize(self, event):
        bmp = self.impil.Scale(self.)
        self.bmwx.SetBitmap(wx.BitmapFromImage(bmp))
        self.Refresh()
        self.Layout()
    
    def get_bitmap( self, np_image, width=32, height=32, colour = (0,0,0) ):
        image = cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)
        array = np.zeros( (height, width, 3),'uint8')
        array[:,:,] = colour
        image = wx.EmptyImage(width,height)
        image.SetData( array.tostring())
        wxBitmap = image.ConvertToBitmap()       # OR:  wx.BitmapFromImage(image)
        return wxBitmap

    def scale_bitmap(self, bitmap, width, height):
        image = wx.ImageFromBitmap(bitmap)
        image = image.Scale(width, height, wx.IMAGE_QUALITY_HIGH)
        result = wx.BitmapFromImage(image)
        return result

    def onResize(self, event):
        # self.Layout()
        frame_size = self.GetSize()
        frame_h = (frame_size[0]-10) / 2
        frame_w = (frame_size[1]-10) / 2
        bmp = self.png.Scale(frame_h,frame_w)
        self.bmp.SetBitmap(wx.BitmapFromImage(bmp))'''

class MainWindow(wx.Frame):
    def __init__(self, parent, title = "monoDrive Visualizer", *args, **kwargs):
        wx.Frame.__init__(self, parent, size=(1200,1000), title = title,*args, **kwargs)
        #set up frame panels
        #self.top_row_panel = TopRow(self,wx.ID_ANY,style=wx.CLIP_CHILDREN)
        self.graph_row_panel = GraphRow(self)
        self.text_row_panel = TextRow(self)
        self.camera_row_panel = CameraRow(self)

        #set up sizers for frame panels
        self.main_sizer = wx.BoxSizer(wx.VERTICAL)
        self.main_sizer.Add(self.graph_row_panel,0, wx.ALL|wx.EXPAND, border = 2)
        self.main_sizer.Add(self.text_row_panel, 0, wx.ALL|wx.EXPAND, border = 2)
        self.main_sizer.Add(self.camera_row_panel, 0, wx.ALL|wx.EXPAND, border = 2)
        self.SetSizer(self.main_sizer)

        #add figures to graph row
        self.roadmap_view = RoadMap_View(self.graph_row_panel)
        self.graph_row_panel_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.graph_row_panel_sizer.Add(self.roadmap_view, 1, wx.EXPAND|wx.ALL, border=2)
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
        self.camera_row_panel_sizer.Add(self.camera_view, 1, wx.EXPAND|wx.ALL, border = 2)
        self.camera_row_panel.SetSizerAndFit(self.camera_row_panel_sizer)

        self.Layout()
        self.Refresh()

class MyFrame(wx.Frame):
    pass

class MonoDriveGUIApp(wx.App, wx.lib.mixins.inspection.InspectionMixin):
#class MonoDriveGUIApp(wx.App):
    def OnInit(self):
        self.Init()  # initialize the inspection tool
        frame = MainWindow(None)
        frame.Show(True)
        self.SetTopWindow(frame)
        return True

class GUI(multiprocessing.Process):
    def __init__(self, vehicle, **kwargs):
        super(GUI, self).__init__(**kwargs)
        self.name = "GUI"
        self.vehicle = vehicle
        self.running = True
        self.start()
        #prctl.set_proctitle("mono{0}".format(self.name))

    def run(self):
        
        sensor_poll = SensorPoll(self.vehicle)
        while self.running:
            app = MonoDriveGUIApp()
            app.MainLoop()
        sensor_poll.stop()
    
    def stop(self):
        self.running = False

class SensorPoll(Thread):
    """Thread to pull data from sensor q's and publish to views"""
    def __init__(self, vehicle):
        Thread.__init__(self)
        self.vehicle = vehicle
        self.map = vehicle.get_road_map()
        self.sensors = vehicle.get_sensors()
        self.running = True
        self.start()

    def update_gui(self, sensor):
        
        if "IMU" in sensor.name:
            wx.CallAfter(pub.sendMessage, "update_imu", msg=sensor.get_message())
        if "GPS" in sensor.name:
            wx.CallAfter(pub.sendMessage, "update_gps", msg=sensor.get_message())
        if "Camera" in sensor.name:
            wx.CallAfter(pub.sendMessage, "update_camera", msg=sensor.get_message())

        
    
    #this thread will run while application is running
    def run(self):
        #map is not a sensor but so we call this every loop
        
        while self.running:
            time.sleep(2)
            for sensor in self.sensors:
                self.update_gui(sensor)
            wx.CallAfter(pub.sendMessage, "update_roadmap", msg=self.map)

            
            
    
    def get_test_message(self):
        imu_msg = IMU_Message.test_message()
        return imu_msg

if __name__ == '__main__':
    #vehicle = "vehicle"
    #gui = GUI(vehicle)
    print("running MonoDriveGUIApp")
    app = MonoDriveGUIApp()
    if app:
        app.MainLoop()
    else:
        print("MonoDriveGUI app not running")