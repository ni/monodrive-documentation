
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import struct
try:
    from tkinter import *
except ImportError:
    from Tkinter import *

from . import BaseSensor
from .gui import TkinterSensorUI


class RPM(TkinterSensorUI, BaseSensor):
    def __init__(self, idx, config, simulator_config, **kwargs):
        super(RPM, self).__init__(idx=idx, config=config, simulator_config=simulator_config, **kwargs)
        self.wheel_number = None
        self.wheel_rpm = None

    @classmethod
    def parse_frame(cls, frame, time_stamp, game_time):
        wheel_number, wheel_speed = list(struct.unpack('=if', frame))
        data_dict = {
            'time_stamp': time_stamp,
            'game_time': game_time,
            'wheel_number': wheel_number,
            'wheel_rpm': wheel_speed
        }
        return data_dict

    def initialize_views(self):
        self.view_lock.acquire()

        super(RPM, self).initialize_views()

        self.string_wheel_number = StringVar()
        self.wheel_number_text_display = Label(self.master_tk, textvariable=self.string_wheel_number)
        self.wheel_number_text_display.pack()

        self.string_wheel_rpm = StringVar()
        self.wheel_rpm_text_display = Label(self.master_tk, textvariable=self.string_wheel_rpm)
        self.wheel_rpm_text_display.pack()

        self.string_timestamp = StringVar()
        self.timestamp_text_display = Label(self.master_tk, textvariable=self.string_timestamp)
        self.timestamp_text_display.pack()

        self.view_lock.release()

    def process_display_data(self):
        data = self.q_display.get()
        self.string_wheel_number.set('Wheel Number: {0}'.format(data['wheel_number']))
        self.string_wheel_rpm.set('RPM: {0}'.format(data['wheel_rpm']))
        self.string_timestamp.set('Time Stamp: {0}'.format(data['time_stamp']))
        self.update_sensors_got_data_count()
