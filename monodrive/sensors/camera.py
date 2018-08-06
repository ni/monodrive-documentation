
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import logging
import numpy as np
import time

from . import BaseSensorPacketized


class Camera(BaseSensorPacketized):
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

        self.bounding_box = None
        self.b_draw_bounding_boxes = False

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
        image_frame = self.get_message()
        image_buffer = image_frame['image']
        if len(image_buffer) == self.height * self.width * 4:
            image = np.array(bytearray(image_buffer), dtype=np.uint8).reshape(self.height, self.width, 4)
        else:
            image = None
            logging.getLogger("sensor").error("wrong image size received {0}".format(self.name))
        return image

    def process_bound_data(self, data):
        sensor_id_str = str(self.sensor_id)
        self.bounding_box_positions = data['bounding_box_positions'][sensor_id_str]
        self.bounding_box_is_in_camera_fov = data['in_camera_los'][sensor_id_str]


    def process_display_data(self):

        if self.bounding_box:
            self.bounding_box.update_sensors_got_data_count()
        self.update_sensors_got_data_count()

    def stop_sub_processes(self):
        self.update_sensors_got_data_count()

    def stop(self):
        if not self.hdmi_streaming:
            super(Camera, self).stop()


class MultiCamera(Camera):
    def __init__(self, idx, config, simulator_config, **kwargs):
        super(MultiCamera, self).__init__(idx, config, simulator_config=simulator_config, **kwargs)
        self.camera_ids = self.config["camera_ids"]

    def process_bound_data(self, data):
        
        self.bounding_box_positions = []
        self.bounding_box_is_in_camera_fov = []
        i = 0
        for camera_id in self.camera_ids:
            single_camera_width = self.width / len(self.camera_ids)
            x_offset = i * single_camera_width

            bounding_box_positions = data['bounding_box_positions'][camera_id]
            in_line_of_sight_bools = data['in_camera_los'][camera_id]
            for x in range(0, len(bounding_box_positions)):
                positions = bounding_box_positions[x]
                in_line_of_sight = in_line_of_sight_bools[x]

                if self.check_in_bounds(positions):
                    positions[0] += x_offset
                    positions[2] += x_offset
                    positions[4] += x_offset
                    positions[6] += x_offset
                    positions[8] += x_offset
                    positions[10] += x_offset
                    positions[12] += x_offset
                    positions[14] += x_offset

                    self.bounding_box_positions.append(positions)
                    self.bounding_box_is_in_camera_fov.append(in_line_of_sight)

            i = i + 1

    def check_in_bounds(self, positions):
        for i in range(0, len(positions)):
            pos_point = positions[i]
            if pos_point < 0 or pos_point > 512:
                return False

        return True