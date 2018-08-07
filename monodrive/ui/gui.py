#!/usr/bin/env python
import wx
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

#For image getting
from os import path
basepath = path.dirname(__file__)
filepath = path.abspath(path.join(basepath, "Capture.png"))
import numpy
#f = open(filepath, "r")

import multiprocessing
from threading import Thread
from wx.lib.pubsub import pub
import time

from message import IMU_Message

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

        pub.subscribe(self.update_view, "update")

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

class Camera_View(wx.Image):
    def __init__(self, parent):
        wx.Image.__init__(self,filepath, wx.BITMAP_TYPE_ANY)
        self.png = wx.Image(filepath, wx.BITMAP_TYPE_ANY)
        self.bmp = wx.StaticBitmap(parent, wx.ID_ANY, wx.BitmapFromImage(self.png),(0, 0), (self.png.GetWidth()/2, self.png.GetHeight()/2))
    
    def get_bmp(self):
        return self.bmp

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
        #TODO update this in the main loop
        #self.Refresh()
        #self.Layout()

    #Not currently used
    def UpdateImage2D(self):
		image_array = self.object
		imagedata = numpy.array(image_array, dtype=numpy.double)
		imagedata[imagedata < 1e-6] = 1.0
		imagedata = numpy.log(imagedata)
		imagedata = imagedata - imagedata.min()
		if imagedata.max() > 0:
			imagedata = (255.0/imagedata.max())*imagedata
		else:
			imagedata = 255.0*imagedata
		imagedatalow = numpy.uint8(imagedata)
		self.impil = wx.Image.fromarray(imagedatalow, 'L').resize((self.sx,self.sy))
		self.imwx = wx.EmptyImage( self.impil.size[0], self.impil.size[1] )
		self.imwx.SetData( self.impil.convert( 'RGB' ).tobytes() )
		bitmap = wx.BitmapFromImage(self.imwx)
		self.bmp = bitmap
		self.image.SetBitmap(bitmap)
		self.Refresh()
		self.Layout() 

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

class SensorPoll(Thread):
    """Thread to pull data from sensor q's and publish to views"""
    def __init__(self, vehicle):
        Thread.__init__(self)
        self.vehicle = vehicle
        self.start()

    #this thread will run while application is running
    def run(self):
        for sample in range(100):
            time.sleep(1)
            wx.CallAfter(pub.sendMessage, "update", msg=self.get_message())
    
    def get_message(self):
        imu_msg = IMU_Message.test_message()
        return imu_msg

if __name__ == '__main__':
    vehicle = "vehicle"
    gui = GUI(vehicle)