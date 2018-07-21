import struct

try:
    from tkinter import *
except ImportError:
    from Tkinter import *

from . import BaseSensor
from .gui import TkinterSensorUI


class IMU(TkinterSensorUI, BaseSensor):
    def __init__(self, idx, config, simulator_config, **kwargs):
        super(IMU, self).__init__(idx=idx, config=config, simulator_config=simulator_config, **kwargs)
        self.framing = None
        self.acceleration_vector = None
        self.angular_velocity_vector = None
        self.timer = None

    @classmethod
    def parse_frame(cls, frame, time_stamp, game_time):
        fmt = '=ffffffih'
        accel_x, accel_y, accel_z, ang_rate_x, ang_rate_y, ang_rate_z, timer, check_sum = list(
            struct.unpack(fmt, frame[1:31]))
        acceleration_vector = [accel_x, accel_y, accel_z]
        angular_velocity_vector = [ang_rate_x, ang_rate_y, ang_rate_z]
        data_dict = {
            'time_stamp': time_stamp,
            'game_time': game_time,
            'acceleration_vector': acceleration_vector,
            'angular_velocity_vector': angular_velocity_vector,
            'timer': timer
        }
        return data_dict

    def initialize_views(self):
        self.view_lock.acquire()

        super(IMU, self).initialize_views()

        self.string_accel_x = StringVar()
        self.accel_x_text_display = Label(self.master_tk, textvariable=self.string_accel_x)
        self.accel_x_text_display.pack()

        self.string_accel_y = StringVar()
        self.accel_y_text_display = Label(self.master_tk, textvariable=self.string_accel_y)
        self.accel_y_text_display.pack()

        self.string_accel_z = StringVar()
        self.accel_z_text_display = Label(self.master_tk, textvariable=self.string_accel_z)
        self.accel_z_text_display.pack()

        self.string_ang_rate_x = StringVar()
        self.ang_rate_x_text_display = Label(self.master_tk, textvariable=self.string_ang_rate_x)
        self.ang_rate_x_text_display.pack()

        self.string_ang_rate_y = StringVar()
        self.ang_rate_y_text_display = Label(self.master_tk, textvariable=self.string_ang_rate_y)
        self.ang_rate_y_text_display.pack()

        self.string_ang_rate_z = StringVar()
        self.ang_rate_z_text_display = Label(self.master_tk, textvariable=self.string_ang_rate_z)
        self.ang_rate_z_text_display.pack()

        self.string_timer = StringVar()
        self.timer_text_display = Label(self.master_tk, textvariable=self.string_timer)
        self.timer_text_display.pack()

        self.view_lock.release()

    def process_display_data(self):
        data = self.q_display.get()
        self.string_accel_x.set('ACCEL_X: {0}'.format(data['acceleration_vector'][0]))
        self.string_accel_y.set('ACCEL_Y: {0}'.format(data['acceleration_vector'][1]))
        self.string_accel_z.set('ACCEL_Z: {0}'.format(data['acceleration_vector'][2]))
        self.string_ang_rate_x.set('ANG RATE X: {0}'.format(data['angular_velocity_vector'][0]))
        self.string_ang_rate_y.set('ANG RATE Y: {0}'.format(data['angular_velocity_vector'][1]))
        self.string_ang_rate_z.set('ANG RATE X: {0}'.format(data['angular_velocity_vector'][2]))
        self.string_timer.set('TIMESTAMP: {0}'.format(data['time_stamp']))
        self.update_sensors_got_data_count()
