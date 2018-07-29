
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import logging

from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
try:
    from tkinter import *
except ImportError:
    from Tkinter import *

import cv2
import json
import matplotlib
import multiprocessing
import os
import threading


matplotlib.use('TkAgg')


class BaseSensorUI(object):
    Window_Settings_Lock = multiprocessing.Lock()

    def __init__(self, **kwargs):
        super(BaseSensorUI, self).__init__(**kwargs)
        self.view_lock = None
        self.process_data_thread = None
        self.window_x_position = 0
        self.window_y_position = 0
        self.view_changing_timer = None
        self.previous_event = None
        self.b_stop_thread = False
        

    def initialize_views(self):
        # override for UI creation
        return

    def render_views(self):
        self.process_data_thread.join()
        return

    def process_display_data(self):
        # override for main data processing
        return

    def rendering_main(self):
        self.view_lock = threading.Lock()
        self.initialize_views()
        thread_name = self.name + 'Process_Data_Thread'
        self.process_data_thread = threading.Thread(target=self.process_data_loop, args=(self,), name=thread_name)
        self.process_data_thread.start()
        self.render_views()
    
    def stop_rendering(self):
        logging.getLogger("sensor").info("shutting down rendering thread: {0}".format(self.name))
        if self.process_data_thread is not None:
            self.process_data_thread.stop()
        else:
            logging.getLogger("sensor").info("no thread: {0}".format(self.name))
        
    #@staticmethod
    #def stop(self):
    #    logging.getLogger("sensor").info("shutting down rendering thread: {0}".format(self.name))
    #    if self.process_data_thread != None:
    #        self.process_data_thread.stop()
    #    else:
    #        logging.getLogger("sensor").info("no thread: {0}".format(self.name))

    def set_window_coordinates(self, window_settings):
        if self.name in window_settings:
            coords = window_settings[self.name]
            self.window_x_position = coords['x']
            self.window_y_position = coords['y']

    def save_window_settings(self):
        BaseSensorUI.Window_Settings_Lock.acquire()
        current_settings = {}
        if os.path.exists('window_settings.json'):
            with open('window_settings.json') as data_file:
                current_settings = json.load(data_file)
        current_settings[self.name] = {
            'x': self.window_x_position,
            'y': self.window_y_position
        }
        with open('window_settings.json', 'w') as outfile:
            json.dump(current_settings, outfile)
        BaseSensorUI.Window_Settings_Lock.release()

    def window_configure_event(self, event):
        """ Event that fires when the window changes position. """
        if self.previous_event is not None:
            if self.previous_event.x == event.x and self.previous_event.y == event.y:
                return

        self.previous_event = event

        # print("window_configure_event({0},{1}) -> ({2},{3}) - {4}".format(self.window_x_position,
        #                                                                   self.window_y_position, event.x, event.y,
        #                                                                   self.name))
        if self.view_changing_timer is not None or event.x != self.window_x_position or event.y != self.window_y_position:

            self.view_lock.acquire()
            if self.view_changing_timer is not None:
                self.view_changing_timer.cancel()
            self.view_changing_timer = threading.Timer(.5, self.window_position_changed, args=(event,))
            self.view_changing_timer.start()
            self.view_lock.release()

    def window_position_changed(self, event):
        # print("window_position_changed({0},{1}) -> ({2},{3}) - {4}".format(self.window_x_position, self.window_y_position, event.x, event.y, self.name))
        self.view_lock.acquire()
        self.view_changing_timer.cancel()
        self.view_changing_timer = None
        self.view_lock.release()
        if event.x != self.window_x_position or event.y != self.window_y_position:
            if hasattr(self, 'window_x_position'):
                self.window_x_position = event.x
            if hasattr(self, 'window_y_position'):
                self.window_y_position = event.y
            self.previous_event = None
            self.save_window_settings()

    @property
    def window_configuration_coordinates(self):
        return str(self.window_x_position) + '+' + str(self.window_y_position)

    # Render thread entry point
    @staticmethod
    def process_data_loop(sensor):
        while sensor.running:
            sensor.process_display_data()


class MatplotlibSensorUI(BaseSensorUI):
    def __init__(self, **kwargs):
        super(MatplotlibSensorUI, self).__init__(**kwargs)
        self.animation = None
        self.main_plot = None

    def initialize_views(self):
        self.main_plot = plt.figure(10)
        plt.get_current_fig_manager().window.bind('<Configure>', self.window_configure_event)
        geometry = "+" + self.window_configuration_coordinates
        plt.get_current_fig_manager().window.wm_geometry(geometry)

    def render_views(self):
        self.animation = FuncAnimation(self.main_plot, self.update_views, interval=100)
        plt.show()

    def update_views(self, frame):
        return


class TkinterSensorUI(BaseSensorUI):
    def __init__(self, **kwargs):
        super(TkinterSensorUI, self).__init__(**kwargs)
        self.master_tk = None

    def initialize_views(self):
        self.master_tk = Tk()
        geometry = "300x200+" + self.window_configuration_coordinates
        self.master_tk.geometry(geometry)
        if hasattr(self, 'name'):
            self.master_tk.title(self.name)
        self.master_tk.bind('<Configure>', self.window_configure_event)

    def render_views(self):
        mainloop()

    def stop_rendering(self):
        # override in subclass
        logging.getLogger("sensor").info("***{0}".format(self.name))
        if self.master_tk != None:
            self.master_tk.destroy()


    def window_configure_event(self, event):
        """ Event that fires when the window changes position. """
        event = Event()
        event.x = self.master_tk.winfo_x()
        event.y = self.master_tk.winfo_y()
        super(TkinterSensorUI, self).window_configure_event(event)
