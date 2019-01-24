#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

from monodrive.ui.radarview import *
from monodrive.ui.boundboxview import *
from monodrive.ui.roadmapview import *
from monodrive.ui.gpsview import *
from monodrive.ui.imuview import *
from monodrive.ui.rpmview import *
from monodrive.ui.cameraview import *
import math


class Overview_Panel(wx.Panel):
    def __init__(self, parent, cameras, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)
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
        self.camera_views = []
        self.camera_row_panel_sizer = wx.BoxSizer(wx.HORIZONTAL)
        for camera_name in cameras:
            self.camera_views.append(Camera_View(self.camera_row_panel, camera_name))
            self.camera_row_panel_sizer.Add(self.camera_views[-1], 1, wx.EXPAND | wx.ALL, border=2)
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

        min_h = min(Size.x / 3, h)

        self.roadmap_view.SetMinSize(wx.Size(min_h, min_h))
        self.radar_polar_plot.SetMinSize(wx.Size(min_h, min_h))
        self.bounding_box_plot.SetMinSize(wx.Size(min_h, min_h))

        self.graph_row_panel.SetMinSize(wx.Size(Size.x, h + GraphHeaders))
        self.camera_row_panel.SetMinSize(wx.Size(Size.x, h))

        self.graph_row_panel.Layout()
        self.Layout()


class CameraPanel(wx.Panel):
    def __init__(self, parent, cameras, *args, **kwargs):
        wx.Panel.__init__(self, parent, *args, **kwargs)

        num_cameras = len(cameras)
        if num_cameras == 0:
            return

        num_rows = int(math.sqrt(num_cameras))
        nominal_col_length = math.ceil(num_cameras/num_rows)

        gs = wx.GridSizer(rows=num_rows, cols=nominal_col_length, vgap=2, hgap=2)
        for camera in cameras:
            gs.Add(Camera_View(self, camera), 1, wx.EXPAND)
        self.SetSizer(gs)
        self.SetBackgroundColour(BACKGROUND_COLOR)


class MainFrame(wx.Frame):
    def __init__(self, cameras):
        width = int(wx.SystemSettings.GetMetric(wx.SYS_SCREEN_X) * .9)
        height = int(wx.SystemSettings.GetMetric(wx.SYS_SCREEN_Y) * .9)
        wx.Frame.__init__(self, None, size=(width, height), title = "monoDrive Visualizer")
        pub.subscribe(self.shutdown, "SHUTDOWN")
        # Here we create a panel and a notebook on the panel
        p = wx.Panel(self)
        nb = wx.Notebook(p)

        # create the page windows as children of the notebook
        overview = Overview_Panel(nb, cameras)
        radar = Radar_Panel(nb)
        #camera = Camera_View(nb, "Camera")
        camera = CameraPanel(nb, cameras)

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
        self.Close()


#Ctrl-Alt-I, or Cmd-Alt-I on Mac for inspection app for viewing layout
#class MonoDriveGUIApp(wx.App, wx.lib.mixins.inspection.InspectionMixin):
class MonoDriveGUIApp(wx.App):
    #def OnInit(self, num_cameras):
    def __init__(self, cameras):
        wx.App.__init__(self)
        #self.Init()  # initialize the inspection tool
        self.MainWindow = MainFrame(cameras)
        self.MainWindow.Show(True)
        self.SetTopWindow(self.MainWindow)
        #wx.lib.inspection.InspectionTool().Show()
        #return True

    def Close(self):
        try:
            wx.CallAfter(self.MainWindow.Close)
            time.sleep(0.2)
        except: pass
        wx.CallAfter(self.Destroy)
        time.sleep(0.2)


class SensorPoll(Thread):
    """Thread to pull data from sensor q's and publish to views"""
    def __init__(self, sensors, fps, map, clock_mode, vehicle_step_event):
        super(SensorPoll,self).__init__()
        #self.vehicle = vehicle
        #TODO need to fix the map getting here
        #self.road_map = vehicle.get_road_map()
        self.road_map = map
        #self.sensors = vehicle.get_sensors()
        self.sensors = sensors
        self.stop_event = multiprocessing.Event()
        self.vehicle_step_event = vehicle_step_event
        self.clock_mode = clock_mode
        self.fps = fps
        self.update_gui_rate = 1.0 / float(fps)
        self.start()

    def update_sensor_widget(self, sensor):

        messages = sensor.get_display_messages(block=True, timeout=0.01)
        found_data = len(messages) > 0
        if found_data:
            message = messages.pop() #pop last message aka. most recent
            if "IMU" in sensor.name:
                wx.CallAfter(pub.sendMessage, "update_imu", msg=message)
            elif "GPS" in sensor.name:
                wx.CallAfter(pub.sendMessage, "update_gps", msg=message)
            elif "Camera" in sensor.name:
                message['width'] = sensor.width
                message['height'] = sensor.height
                wx.CallAfter(pub.sendMessage, sensor.name, msg=message)
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
            should_stop = self.stop_event.wait(self.update_gui_rate)
            updated = self.update_sensor_widget(sensor)
            wait_time += self.update_gui_rate

        if not updated:
            logging.getLogger("gui").debug("gui update skipped - sensor data not received (mode: ClockMode_ClientStep, stop_event = %s): %s"
                                           % (should_stop, sensor.name))
        return should_stop

    def update_gui(self, sensor):
        updated = self.update_sensor_widget(sensor)
        if self.clock_mode == ClockMode_ClientStep:
            if not updated and self.check_client_mode_update(sensor):
                return False
        return True

    #this thread will run while application is running
    def run(self):
        while not self.stop_event.wait(self.update_gui_rate):
            for sensor in self.sensors:
                if not self.update_gui(sensor):
                    break

            if self.clock_mode == ClockMode_ClientStep and self.vehicle_step_event is not None:
                self.vehicle_step_event.set()

            if self.road_map:
                wx.CallAfter(pub.sendMessage, "update_roadmap", msg=self.road_map)

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
            logging.getLogger("gui").debug('Lidar connected to {0}'.format(str((VELOVIEW_PORT, VELOVIEW_PORT))))
            return s
        except Exception as e:
            logging.getLogger("gui").debug('Cannot connect to {0}'.format(str((VELOVIEW_PORT, VELOVIEW_PORT))))
            logging.getLogger("gui").debug("Error {0}".format(e))
            self.veloview_socket = None
            return None


class GUI(object):
    def __init__(self, ego_vehicle, simulator, **kwargs):
        super(GUI, self).__init__(**kwargs)
        self.daemon = True
        self.name = "GUI"
        self.simulator_event = simulator.restart_event
        self.vehicle_step_event = ego_vehicle.vehicle_step_event if ego_vehicle else None
        self.vehicle_clock_mode = ego_vehicle.vehicle_config.clock_mode if ego_vehicle else 0
        #for instance in ego_vehicle.sensor_process_dict:
        #    print(instance)
            #print(instance.name)


        #self.simulator = simulator
        #self.vehicle = None
        #self.vehicle = ego_vehicle

        #self.num_cameras = 0
        self.cameras = []
        self.sensors = []
        for sensor in ego_vehicle.sensors:
            if "Lidar" in sensor.name:
                self.sensors.append(LidarGUISensor(sensor))
            else:
                self.sensors.append(GUISensor(sensor))
            if "Camera" in sensor.name:
                self.cameras.append(sensor.name)
                #self.num_cameras = self.num_cameras + 1

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
            logging.getLogger("gui").debug("could not join process {0} -> {1}".format(self.name, e))

    def run(self):

        monitor = Thread(target=self.monitor_process_state)
        monitor.start()

        self.app = MonoDriveGUIApp(self.cameras)

        #prctl.set_proctitle("mono{0}".format(self.name))
        #start sensor polling
        self.sensor_polling = SensorPoll(self.sensors, self.fps, self.map, self.vehicle_clock_mode, self.vehicle_step_event)
        while not self.stop_event.is_set():
            self.app.MainLoop()
            self.simulator_event.set()
            time.sleep(0.5)

        monitor.join(timeout=2)


    #this will simply wait for sensor to stop running - and kill the app
    def monitor_process_state(self):
        self.stop_event.wait()
        self.sensor_polling.stop()
        self.app.Close()


if __name__ == '__main__':
    app = MonoDriveGUIApp(1)
    if app:
        app.MainLoop()
    else:
        logging.getLogger("gui").debug("MonoDriveGUI app not running")
