
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import logging
import cv2
import numpy as np
import time
import os

from . import BaseSensorPacketized
from . import BaseSensorPacketized
from .gui import TkinterSensorUI

from PIL import Image
from PIL import ImageTk
try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk


class Camera(TkinterSensorUI, BaseSensorPacketized):
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
        image_frame = self.q_display.get()
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

    def initialize_views(self):
        self.view_lock.acquire()
        super(Camera, self).initialize_views()

        self.panel = tk.Label()
        self.panel.pack(fill=tk.BOTH)

        self.view_lock.release()

    def process_display_data(self):
        if self.display_process:

            if self.bounding_box is not None:
                bounding_data = self.bounding_box.q_vehicle.peek()
                self.process_bound_data(bounding_data)

            if self.hdmi_streaming:
                ret, self.current_image = self.video_capture.read()

                # Assumption 1: Resolution of Sim is set to 1080p.
                stream_size_width = 1920
                stream_size_height = 1080
                # Assumption 2: Standalone sim is 1024x1024.
                standalone_width = 1024
                standalone_height = 1024
                # Assumption 3: Standalone sim is centered.
                # Assumption 4: 4 cameras in a square grid.
                start_x = int((stream_size_width - standalone_width) / 2)
                start_y = int((stream_size_height - standalone_height) / 2)
                self.current_image = self.current_image[start_y:start_y + standalone_height, start_x:start_x + standalone_width]

                self.frame_num += 1
                time_elapsed = time.clock() - self.second_start_time
                if time_elapsed >= 10.0:
                    num_frames = self.frame_num - self.frame_start_num
                    frame_size = 512 * 512 * 4 * 32
                    packet_size = 6400

                    fps = num_frames / time_elapsed
                    mBps = fps * frame_size / 1000000
                    pps = round(fps * frame_size / packet_size)
                    fps = "{:3.2f}".format(fps)
                    mBps = "{:3.2f}".format(mBps)
                    print("{0}:\t {1} MBps | {2} pps | {3} fps".format(self.__class__.__name__, mBps, pps, fps))

                    self.second_start_time = time.clock()
                    self.frame_start_num = self.frame_num
            else:
                self.current_image = self.get_q_image()

                image = self.current_image
                if self.bounding_box is not None and self.b_draw_bounding_boxes:
                    image = self.draw_bounding_boxes(self.current_image)

                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                image = Image.fromarray(image)
                image = ImageTk.PhotoImage(image)

                # Only update window size when new image size
                if self.master_tk.winfo_width() != len(self.current_image[0]) or self.master_tk.winfo_height() != len(self.current_image):
                    window_size = str(len(self.current_image[0])) + 'x' + str(len(self.current_image))
                    self.master_tk.geometry(window_size)
                self.panel.configure(image=image)
                self.panel.image = image

                self.frame_num += 1

        if self.bounding_box:
            self.bounding_box.update_sensors_got_data_count()
        self.update_sensors_got_data_count()

    def draw_bounding_boxes(self, q):
        overlay1 = q.copy()
        overlay2 = q.copy()
        output = q.copy()

        for i in range(0, len(self.bounding_box_positions)):
            pos0_x = self.bounding_box_positions[i][0]
            pos0_y = self.bounding_box_positions[i][1]
            pos1_x = self.bounding_box_positions[i][2]
            pos1_y = self.bounding_box_positions[i][3]
            pos2_x = self.bounding_box_positions[i][4]
            pos2_y = self.bounding_box_positions[i][5]
            pos3_x = self.bounding_box_positions[i][6]
            pos3_y = self.bounding_box_positions[i][7]
            pos4_x = self.bounding_box_positions[i][8]
            pos4_y = self.bounding_box_positions[i][9]
            pos5_x = self.bounding_box_positions[i][10]
            pos5_y = self.bounding_box_positions[i][11]
            pos6_x = self.bounding_box_positions[i][12]
            pos6_y = self.bounding_box_positions[i][13]
            pos7_x = self.bounding_box_positions[i][14]
            pos7_y = self.bounding_box_positions[i][15]
            is_in_fov = self.bounding_box_is_in_camera_fov[i]

            if is_in_fov:
                ''' Draw 3d bounding box in image
                        qs: (8,3) array of vertices for the 3d box in following order:
                            1 -------- 0
                           /|         /|
                          2 -------- 3 .
                          | |        | |
                          . 5 -------- 4
                          |/         |/
                          6 -------- 7
                    '''

                rectangle_color = (255, 0, 0)
                line_width = 2

                pts = np.array([[pos1_x, pos1_y], [pos0_x, pos0_y], [pos3_x, pos3_y], [pos2_x, pos2_y]], np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.fillPoly(overlay1, [pts], rectangle_color)
                cv2.polylines(overlay2, [pts], True, rectangle_color, line_width)

                pts = np.array([[pos5_x, pos5_y], [pos4_x, pos4_y], [pos7_x, pos7_y], [pos6_x, pos6_y]], np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.fillPoly(overlay1, [pts], rectangle_color)
                cv2.polylines(overlay2, [pts], True, rectangle_color, line_width)

                pts = np.array([[pos3_x, pos3_y], [pos0_x, pos0_y], [pos4_x, pos4_y], [pos7_x, pos7_y]], np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.fillPoly(overlay1, [pts], rectangle_color)
                cv2.polylines(overlay2, [pts], True, rectangle_color, line_width)

                pts = np.array([[pos2_x, pos2_y], [pos1_x, pos1_y], [pos5_x, pos5_y], [pos6_x, pos6_y]], np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.fillPoly(overlay1, [pts], rectangle_color)
                cv2.polylines(overlay2, [pts], True, rectangle_color, line_width)

                pts = np.array([[pos2_x, pos2_y], [pos3_x, pos3_y], [pos7_x, pos7_y], [pos6_x, pos6_y]], np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.fillPoly(overlay1, [pts], rectangle_color)
                cv2.polylines(overlay2, [pts], True, rectangle_color, line_width)

                pts = np.array([[pos1_x, pos1_y], [pos0_x, pos0_y], [pos4_x, pos4_y], [pos5_x, pos5_y]], np.int32)
                pts = pts.reshape((-1, 1, 2))
                cv2.fillPoly(overlay1, [pts], rectangle_color)
                cv2.polylines(overlay2, [pts], True, rectangle_color, line_width)

        alpha = 0.45
        cv2.addWeighted(overlay1, alpha, overlay2, 1 - alpha, 0, overlay2)
        alpha = 0.50
        cv2.addWeighted(overlay2, alpha, output, 1 - alpha, 0, output)

        return output

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