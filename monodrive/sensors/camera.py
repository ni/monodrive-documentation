
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import logging
import numpy as np
import time
try:
    import cPickle as pickle
except ImportError:
    import _pickle as pickle

from .base_sensor import BaseSensor


class Camera(BaseSensor):
    def __init__(self, idx, config, simulator_config, **kwargs):
        super(Camera, self).__init__(idx=idx, config=config, simulator_config=simulator_config, **kwargs)
        if self.packetizer_process:
            self.packetizer_process.packetizer.should_patch_frame = True
        self.resolution = self.config["stream_dimensions"]
        self.width = int(self.resolution['x'])
        self.height = int(self.resolution['y'])
        self.hdmi_streaming = self.config["hdmi_streaming"]
        self.cap = None

        self.frame_size = self.width * self.height * 8 * 4 / 10
        self.frame_num = 0
        self.second_start_time = time.clock()
        self.frame_start_num = 0

        self.video_capture = None
        self.current_image = None

    def run(self):
        if not self.hdmi_streaming:
            super(Camera, self).run()

    def get_frame_size(self):
        return self.frame_size

    @classmethod
    def parse_frame(cls, frame, time_stamp, game_time):
        data_dict = {
            'image': frame,
            'game_time': game_time,
            'time_stamp': time_stamp
        }
        return data_dict

    def get_q_image(self):
        image_frame = self.q_data.get()
        
        if "SHUTDOWN" in image_frame:
            #self.running = False
            return "SHUTDOWN"

        image_buffer = image_frame['image']
        if len(image_buffer) == self.height * self.width * 4:
            image = np.array(bytearray(image_buffer), dtype=np.uint8).reshape(self.height, self.width, 4)
        else:
            image = None
            logging.getLogger("sensor").error("wrong image size received {0}".format(self.name))
        return image

    def get_message(self, timeout = None):

        image_frame = super(Camera, self).get_message()

        image_buffer = image_frame['image']
        if len(image_buffer) == self.height * self.width * 4:
            image = np.array(bytearray(image_buffer), dtype=np.uint8).reshape(self.height, self.width, 4)
        else:
            image = None
            logging.getLogger("sensor").error("wrong image size received {0}".format(self.name))
        return pickle.dumps(image, protocol=-1)

    def get_display_messages(self, block=True, timeout=None):
        image_frame_list = []
        image_frames = super(Camera, self).get_display_messages(block=block, timeout=timeout)

        for image_frame in image_frames:
            image_buffer = bytearray(image_frame['image'])
            if len(image_buffer) == self.height * self.width * 4:
                image = np.array(image_buffer, dtype=np.uint8).reshape(self.height, self.width, 4)
            else:
                image = None
                logging.getLogger("sensor").error("wrong image size received {0}".format(self.name))
            image_frame_list.append(image)
        
        return image_frame_list

    def stop(self):
        if not self.hdmi_streaming:
            super(Camera, self).stop()


class MultiCamera(Camera):
    def __init__(self, idx, config, simulator_config, **kwargs):
        super(MultiCamera, self).__init__(idx, config, simulator_config=simulator_config, **kwargs)
        self.camera_ids = self.config["camera_ids"]