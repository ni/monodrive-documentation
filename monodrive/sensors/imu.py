
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import struct


from . import BaseSensor


class IMU(BaseSensor):
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

    def process_display_data(self):
        return
        data = self.q_display.get()
        self.update_sensors_got_data_count()
