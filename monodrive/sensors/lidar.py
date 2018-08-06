
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

from multiprocessing import Queue
import socket

from monodrive.constants import VELOVIEW_PORT, VELOVIEW_IP
from . import BaseSensor


class Lidar(BaseSensor):
    def __init__(self, idx, config, simulator_config, **kwargs):
        super(Lidar, self).__init__(idx=idx, config=config, simulator_config=simulator_config, **kwargs)

        self.drop_frames = False
        channels_per_block = 32
        blocks_per_packet = 12
        number_blocks = 360 / config['horizontal_resolution'] * config['n_lasers'] / channels_per_block
        number_packets =  number_blocks / blocks_per_packet
        self.packet_size = int(number_blocks * 1248.0)
        self.frame_buffer = []
        self.expected_frames_per_step = number_packets

    @classmethod
    def init_data_queue(cls):
        return Queue()

    def digest_frame(self, frame, time_stamp, game_time):
        self.frame_buffer.append(frame)
        if len(self.frame_buffer) == self.expected_frames_per_step:
            temp_buffer = self.frame_buffer
            self.frame_buffer = []
            super(Lidar, self).digest_frame(temp_buffer, time_stamp, game_time)

            
