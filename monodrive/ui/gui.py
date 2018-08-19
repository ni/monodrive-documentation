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

import time

from monodrive.models import IMU_Message
from monodrive.models import GPS_Message
from monodrive.models import Camera_Message
from monodrive.models import MapData
from monodrive.models import Radar_Message


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
        self.max_number_of_targets = 20

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
        self.Layout()
        self.Refresh()

    def setup_radar_plots(self, targets={}):
        targets_cells = np.array(self.max_number_of_targets * [6 * [0]])
        targets_handle = self.target_table_subplot.table(cellText=targets_cells[0:self.max_number_of_targets],
                                          colLabels=("Target", "Range", "Speed", "AoA", "RCS", "Power\n level"),
                                          loc='center')
        targets_handle.auto_set_font_size(False)
        targets_handle.set_fontsize(5.5)
        for i in range(0,6):
            for j in range(len(targets.ranges) + 1, self.max_number_of_targets + 1):
                    targets_handle._cells[(j,i)]._text.set_text('')
                    targets_handle._cells[(j,i)].set_linewidth(0)

        self.old_size = len(targets.ranges)

        return targets_handle

    def set_data(self,targets):
        target_cells = np.array(self.max_number_of_targets * [6 * [0]])
        if len(targets.ranges):
            target_cells[0:len(targets.ranges),    0] = range(0, len(targets.ranges))
            target_cells[0:len(targets.ranges),     1] = targets.ranges
            target_cells[0:len(targets.velocities), 2] = targets.velocities
            target_cells[0:len(targets.aoa_list),   3] = targets.aoa_list
            target_cells[0:len(targets.rcs_list),   4] = targets.rcs_list
            target_cells[0:len(targets.power_list), 5] = targets.power_list

        for i in range(0, 6):
            for j in range(1, self.old_size + 1): #to erase previous display
                try:
                    self.target_table_handle._cells[(j,i)]._text.set_text('')
                    self.target_table_handle._cells[(j,i)].set_linewidth(0)
                except:
                    pass

        for i in range(0,6):
            for j in range(1, len(targets.ranges) + 1): #to refresh with new display
                try:
                    self.target_table_handle._cells[(j, i)]._text.set_text(target_cells[j - 1, i])
                    self.target_table_handle._cells[(j, i)].set_linewidth(1)
                except:
                    pass

        self.old_size = len(targets.ranges)


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
        print("update camera view")
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
        pub.subscribe(self.shutdown, "SHUTDOWN")

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
        self.radar_target_table = Radar_Target_Table(self.graph_row_panel)

        self.graph_row_panel_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.graph_row_panel_sizer.Add(self.roadmap_view, 1, wx.EXPAND|wx.ALL, border=2)
        self.graph_row_panel_sizer.Add(self.radar_target_table, 1, wx.EXPAND|wx.ALL, border=2)
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
        
    
    def shutdown(self, msg):
        print("Frame shutting down {0}".format(msg))
        self.Close()
        #self.Destroy()

class MyFrame(wx.Frame):
    pass
#Ctrl-Alt-I, or Cmd-Alt-I on Mac for inspection app for viewing layout
class MonoDriveGUIApp(wx.App, wx.lib.mixins.inspection.InspectionMixin):
#class MonoDriveGUIApp(wx.App):
    def OnInit(self):
        self.Init()  # initialize the inspection tool
        frame = MainWindow(None)
        frame.Show(True)
        self.SetTopWindow(frame)
        return True

class SensorPoll(Thread):
    """Thread to pull data from sensor q's and publish to views"""
    def __init__(self, vehicle, fps, map):
        super(SensorPoll,self).__init__()
        self.vehicle = vehicle
        #TODO need to fix the map getting here
        #self.road_map = vehicle.get_road_map()
        self.road_map = map
        self.sensors = vehicle.get_sensors()
        self.running = True
        self.fps = fps
        self.start()

    def update_gui(self, sensor):
        
        print("{0}.get_display_messages()".format(sensor.name))
        messages = sensor.get_display_messages()
        if messages:
            message = messages.pop() #pop last message aka. most recent 
            if "IMU" in sensor.name:
                wx.CallAfter(pub.sendMessage, "update_imu", msg=message)
            elif "GPS" in sensor.name:
                wx.CallAfter(pub.sendMessage, "update_gps", msg=message)
            elif "Camera" in sensor.name:
                wx.CallAfter(pub.sendMessage, "update_camera", msg=message)
            elif "Radar" in sensor.name:
                wx.CallAfter(pub.sendMessage, "update_radar_table", msg=message)
            return True       
        return False

    #this thread will run while application is running
    def run(self):
        while self.running:
            #self.road_map = self.vehicle.get_road_map()
            #print("GUI THREAD RUNNING")
            for sensor in self.sensors:
                self.running = self.update_gui(sensor)
            if self.running == False:
                print("GUI THREAD STOPPED")
            if self.road_map:
                wx.CallAfter(pub.sendMessage, "update_roadmap", msg=self.road_map)
            time.sleep(.1)
        self.running = False     
        print("GUI THREAD NOT RUNNING") 

class GUI(multiprocessing.Process):
    def __init__(self, simulator, **kwargs):
        super(GUI, self).__init__(**kwargs)
        self.daemon = True
        self.name = "GUI"
        self.simulator = simulator
        self.vehicle = simulator.ego_vehicle
        self.running = True
        self.app = None
        self.fps = None
        self.settings = simulator.configuration.gui_settings
        self.init_settings(self.settings)
        self.map = simulator.map_data
        self.start()
    
    def init_settings(self, settings):
        default_update_rate = 1.0
        if settings:
            self.settings = settings
            self.fps = self.settings['fps']
        else:
            self.fps = default_update_rate

    def run(self):
        #prctl.set_proctitle("mono{0}".format(self.name))
        #start sensor polling
        self.sensor_polling = SensorPoll(self.vehicle, self.fps, self.map)
        while True:
            self.app = MonoDriveGUIApp()
            self.app.MainLoop()

    def stop(self):
        self.running = False
        #self.sensor_polling.join()
        self.join()


if __name__ == '__main__':
    #vehicle = "vehicle"
    #gui = GUI(vehicle)
    print("running MonoDriveGUIApp")
    app = MonoDriveGUIApp()
    if app:
        app.MainLoop()
    else:
        print("MonoDriveGUI app not running")