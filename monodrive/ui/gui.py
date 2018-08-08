#!/usr/bin/env python
import wx
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

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
from wx.lib.pubsub import pub
import time

from message import IMU_Message
from message import GPS_Message
from message import Camera_Message

BACKGROUND_COLOR = '#eaf7ff'
INNER_PANEL_COLOR = '#f0f0f0'
BLACK = '#000000'
WHITE = '#ffffff'


class TopRow(wx.Panel):
    def __init__(self, parent, id, **kargs):
        wx.Panel.__init__(self, parent=parent)
        self.SetBackgroundColour(BACKGROUND_COLOR)

class CenterRow(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent=parent)
        self.SetBackgroundColour(BLACK)

class BottomRow(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent=parent)
        self.SetBackgroundColour(BLACK)

class GPS_View(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent=parent)
        self.SetBackgroundColour(INNER_PANEL_COLOR)

        self.string_lat = wx.StaticText(self, label="")
        self.string_lng = wx.StaticText(self, label="")
        self.string_time = wx.StaticText(self, label="")

        self.sizer = wx.GridBagSizer(3,1)
        self.sizer.Add(self.string_lat, (0,0))
        self.sizer.Add(self.string_lng, (1,0))
        self.sizer.Add(self.string_time, (2,0))
        self.SetSizerAndFit(self.sizer)

        pub.subscribe(self.update_view, "update_gps")

    def update_view(self, msg):    
        self.string_lat.SetLabelText('LAT: {0}'.format(msg['lat']))
        self.string_lng.SetLabelText('LNG: {0}'.format(msg['lng']))
        self.string_time.SetLabelText('TIMESTAMP: {0}'.format(msg['time_stamp']))


class IMU_View(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent=parent)
        self.SetBackgroundColour(INNER_PANEL_COLOR)

        self.string_accel_x = wx.StaticText(self, label="")
        self.string_accel_y = wx.StaticText(self, label="")
        self.string_accel_z = wx.StaticText(self, label="")
        self.string_ang_rate_x = wx.StaticText(self, label="")
        self.string_ang_rate_y = wx.StaticText(self, label="")
        self.string_ang_rate_z = wx.StaticText(self, label="")
        self.string_timer = wx.StaticText(self, label="")

        self.sizer = wx.GridBagSizer(7, 2)
        self.sizer.Add(self.string_accel_x, (0, 0))
        self.sizer.Add(self.string_accel_y, (1, 0))
        self.sizer.Add(self.string_accel_z, (2, 0))
        self.sizer.Add(self.string_ang_rate_x, (3, 0))
        self.sizer.Add(self.string_ang_rate_y, (4, 0))
        self.sizer.Add(self.string_ang_rate_z, (5, 0))
        self.sizer.Add(self.string_timer, (6, 0))
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
    def __init__(self, parent):
        wx.Panel.__init__(self, parent=parent)
        self.SetBackgroundColour(INNER_PANEL_COLOR)
    def __del__( self ):
        pass

class Camera_View(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent=parent)
        self.png = wx.Image(filepath, wx.BITMAP_TYPE_ANY)
        self.imwx = wx.StaticBitmap(parent, wx.ID_ANY, wx.BitmapFromImage(self.png),(0, 0), (self.png.GetWidth()/2, self.png.GetHeight()/2))
        self.sizer = wx.GridBagSizer(5, 1)
        self.sizer.Add(self.imwx,(1,1))
        self.SetSizer(self.sizer)

        pub.subscribe(self.update_view, "update_camera")

    def update_view(self, msg):
        camera_msg = Camera_Message(msg)
        bitmap = self.to_bmp(camera_msg.np_image)
        self.imwx.SetBitmap(bitmap)

    def convert(self, image_data):
        image = cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(image)

    def to_bmp(self, np_image):
        imcv = cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)
        impil = Image.fromarray(imcv)
        imwx = wx.EmptyImage(impil.size[0], impil.size[1])
        imwx.SetData(impil.convert('RGB').tobytes()) 
        bitmap = wx.BitmapFromImage(imwx)
        return bitmap

    def get_bitmap( self, np_image, width=32, height=32, colour = (0,0,0) ):
        image = cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)
        array = np.zeros( (height, width, 3),'uint8')
        array[:,:,] = colour
        image = wx.EmptyImage(width,height)
        image.SetData( array.tostring())
        wxBitmap = image.ConvertToBitmap()       # OR:  wx.BitmapFromImage(image)
        return wxBitmap

    def get_bmp(self):
        return self.imwx

    def scale_bitmap(self, bitmap, width, height):
        image = wx.ImageFromBitmap(bitmap)
        image = image.Scale(width, height, wx.IMAGE_QUALITY_HIGH)
        result = wx.BitmapFromImage(image)
        return result

    def __del__( self ):
        pass

    def onResize(self, event):
        # self.Layout()
        frame_size = self.GetSize()
        frame_h = (frame_size[0]-10) / 2
        frame_w = (frame_size[1]-10) / 2
        bmp = self.png.Scale(frame_h,frame_w)
        self.bmp.SetBitmap(wx.BitmapFromImage(bmp))


class MainWindow(wx.Frame):
    def __init__(self, parent, id, title = "monoDrive Visualizer"):
        wx.Frame.__init__(self, parent, wx.ID_ANY, title = title, size=(1200,1000))
        self.SetSizeHintsSz( wx.DefaultSize, wx.DefaultSize )
        #set up frame panels
        self.top_row_panel = TopRow(self,wx.ID_ANY,style=wx.CLIP_CHILDREN)
        self.center_row_panel = CenterRow(self)
        self.bottom_row_panel = BottomRow(self)

        #set up sizers for frame panels
        main_sizer = wx.BoxSizer(wx.VERTICAL)
        main_sizer.Add(self.center_row_panel , 3, wx.EXPAND, border = 20)
        main_sizer.Add(self.bottom_row_panel, 3, wx.EXPAND, border = 20)
        main_sizer.Add(self.top_row_panel, 3, wx.EXPAND, border = 20)
        
        
        self.SetSizer(main_sizer)

        #add figures to top row
        self.top_row_panel.figure = Figure()
        self.top_row_panel.canvas = FigureCanvas(self.top_row_panel, 1, self.top_row_panel.figure)
        self.top_row_panel_sizer = wx.BoxSizer()
        self.top_row_panel_sizer.Add(self.top_row_panel.canvas, 1, wx.EXPAND|wx.LEFT|wx.RIGHT|wx.TOP|wx.BOTTOM, border=10)
        self.top_row_panel.SetSizerAndFit(self.top_row_panel_sizer)
        
        #handles to manipulate figure data
        self.axeshandleMap = self.top_row_panel.figure.add_subplot(131,facecolor=INNER_PANEL_COLOR)
        self.axeshandleAOA = self.top_row_panel.figure.add_subplot(132,facecolor=INNER_PANEL_COLOR)
        self.axeshandleTargets = self.top_row_panel.figure.add_subplot(133,facecolor=INNER_PANEL_COLOR)

        #handles to manipulate figure data
        self.gps_view = GPS_View(self.center_row_panel)
        self.imu_view = IMU_View(self.center_row_panel)
        self.wheel_rpm_view = Wheel_RPM_View(self.center_row_panel)

        #add panels to center row and size them
        self.center_row_panel_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.center_row_panel_sizer.Add(self.gps_view, 1, wx.EXPAND|wx.LEFT|wx.RIGHT|wx.TOP|wx.BOTTOM, border=20)
        self.center_row_panel_sizer.Add(self.imu_view, 1, wx.EXPAND|wx.LEFT|wx.RIGHT|wx.TOP|wx.BOTTOM, border=20)
        self.center_row_panel_sizer.Add(self.wheel_rpm_view, 1, wx.EXPAND|wx.LEFT|wx.RIGHT|wx.TOP|wx.BOTTOM, border=20)
        self.center_row_panel.SetSizerAndFit(self.center_row_panel_sizer)

        #add camera to bottom row and size it
        self.camera_view = Camera_View(self.bottom_row_panel)
        self.bmp = self.camera_view.get_bmp()
        self.bottom_row_panel_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.bottom_row_panel_sizer.Add(self.bmp, 1, wx.EXPAND|wx.ALL, border = 20)
        self.bottom_row_panel.SetSizerAndFit(self.bottom_row_panel_sizer)

        self.Layout()
        self.Refresh()
    
class GUI(multiprocessing.Process):
    def __init__(self, vehicle, **kwargs):
        super(GUI, self).__init__(**kwargs)
        self.vehicle = vehicle
        self.running = True
        self.start()

    def run(self):
        sensor_poll = SensorPoll(self.vehicle)
        while self.running:
            app = wx.App(0)
            MainWindow(None, id = wx.ID_ANY).Show()
            app.MainLoop()
        sensor_poll.stop()
    
    def stop(self):
        self.running = False

class SensorPoll(Thread):
    """Thread to pull data from sensor q's and publish to views"""
    def __init__(self, vehicle):
        Thread.__init__(self)
        self.vehicle = vehicle
        self.sensors = vehicle.get_sensors()
        self.running = True
        self.start()

    def update_gui(self, sensor):
        
        if "IMU" in sensor.name:
            wx.CallAfter(pub.sendMessage, "update_imu", msg=sensor.get_message())
            pass
        if "GPS" in sensor.name:
            wx.CallAfter(pub.sendMessage, "update_gps", msg=sensor.get_message())
            pass
        if "Camera" in sensor.name:
            wx.CallAfter(pub.sendMessage, "update_camera", msg=sensor.get_message())
    #this thread will run while application is running
    def run(self):
        
        while self.running:
            for sensor in self.sensors:
                self.update_gui(sensor)
            time.sleep(1)
            
    
    def get_test_message(self):
        imu_msg = IMU_Message.test_message()
        return imu_msg

if __name__ == '__main__':
    vehicle = "vehicle"
    gui = GUI(vehicle)