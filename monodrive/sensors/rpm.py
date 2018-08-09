
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import struct

from . import BaseSensor


class RPM(BaseSensor):
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

    def process_display_data(self):
        return
        data = self.q_display.get()
        self.update_sensors_got_data_count()
