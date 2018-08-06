
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"


import cv2
import json
import logging
import matplotlib
import matplotlib.patches as patches
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from monodrive.constants import VELOVIEW_PORT, VELOVIEW_IP
import multiprocessing
import numpy as np
import os
import threading
try:
    from tkinter import *
except ImportError:
    from Tkinter import *
import socket
import time

from PIL import Image
from PIL import ImageTk


matplotlib.use('TkAgg')

SHOW_MAP = True


class SensorWidget(object):
    def __init__(self, sensor):
        self.sensor = sensor
        self.name = sensor.name if sensor else ''
        self.last_frame = None

    def render(self):
        data = self.sensor.get_message()
        if data is self.last_frame:
            time.sleep(.1)
            return

 #       print("updating {0}".format(self.name))
        self.update_widget(data)
        self.last_frame = data
#        self.sensor.update_sensors_got_data_count()

    def update_widget(self, data):
        raise NotImplemented


class BaseSensorUI(SensorWidget):
    Window_Settings_Lock = multiprocessing.Lock()

    def __init__(self, sensor, **kwargs):
        super(BaseSensorUI, self).__init__(sensor)
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

    def rendering_main(self):
        print("starting render process for {0}".format(self))
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

    # @staticmethod
    # def stop(self):
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
        return "+" + str(self.window_x_position) + '+' + str(self.window_y_position)

    # Render thread entry point
    @staticmethod
    def process_data_loop(widget):
        # prctl.set_proctitle("monodrive rending {0}".format(sensor))
        while widget.sensor.running:
            widget.render()


class MatplotlibSensorUI(BaseSensorUI):
    def __init__(self, sensor, **kwargs):
        super(MatplotlibSensorUI, self).__init__(sensor, **kwargs)
        self.animation = None
        self.main_plot = None

    def initialize_views(self):
        self.main_plot = plt.figure(10)
        plt.get_current_fig_manager().window.bind('<Configure>', self.window_configure_event)
        geometry = self.window_configuration_coordinates
        plt.get_current_fig_manager().window.wm_geometry(geometry)

    def render_views(self):
        self.animation = FuncAnimation(self.main_plot, self.update_views, interval=100)
        plt.show()

    def update_views(self, frame):
        return

    def stop_rendering(self):
        # override in subclass
        logging.getLogger("sensor").info("***{0}".format(self.name))
        if plt != None:
            self.animation.event_source.stop()
            plt.close()
            self.main_plot = None


class TkinterSensorUI(BaseSensorUI):
    def __init__(self, sensor, **kwargs):
        super(TkinterSensorUI, self).__init__(sensor, **kwargs)
        self.master_tk = None

    def initialize_views(self):
        self.master_tk = Tk()
        geometry = "300x200" + self.window_configuration_coordinates
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


class BoundingBoxWidget(MatplotlibSensorUI):
    def __init__(self, sensor, **kwargs):
        super(BoundingBoxWidget, self).__init__(sensor, **kwargs)
        #self.plot = None
        self.patches = []
        self.distances = None
        self.angles = None
        self.x_points = []
        self.y_points = []
        self.x_bounds = None
        self.y_bounds = None
        self.box_rotations = None
        self.velocities = None

    def initialize_views(self):
        self.view_lock.acquire()
        super(BoundingBoxWidget, self).initialize_views()
        self.main_plot.set_size_inches(3.0,2.0)
        self.map_subplot = self.main_plot.add_subplot(111)
#        self.display(x_points, y_points, x_bounds, y_bounds, box_rotations)
        self.map_subplot.set_xlim(-80, 80)
        self.map_subplot.set_ylim(-80, 80)
        self.map_subplot.set_title("Vehicle Targets (birds eye)")
        self.map_subplot.set_xlabel('Range X (m)')
        self.map_subplot.set_ylabel('Range Y (m)')
        self.map_subplot.add_patch(patches.Rectangle((-1.0, -2.0), 2.0, 4.0, color='r'))
        self.view_lock.release()

    def display(self, x_points, y_points, x_bounds, y_bounds, box_rotations):
        for patch in self.patches:
            patch.remove()
        self.patches = []

        if len(x_points) == 0:
            #plt.pause(.001)
            return

        for i in range(0, len(x_points)):
            p = self.map_subplot.add_patch(patches.Rectangle((x_points[i], y_points[i]), x_bounds[i], y_bounds[i],
                                                             angle=box_rotations[i], color='b',fill=False))
            self.patches.append(p)
        #plt.pause(.001)

    def update_views(self, frame):
        if SHOW_MAP:
            self.view_lock.acquire()
            self.display(self.x_points, self.y_points, self.x_bounds, self.y_bounds, self.box_rotations)
            self.view_lock.release()
            return self.map_subplot
        return None

    def update_widget(self, data):
        self.view_lock.acquire()
        self.x_points = data['x_points']
        self.y_points = data['y_points']
        self.x_bounds = data['x_bounds']
        self.y_bounds = data['y_bounds']
        self.box_rotations = data['box_rotations']
        self.view_lock.release()


class CameraWidget(TkinterSensorUI):
    def __init__(self, sensor):
        super(CameraWidget, self).__init__(sensor)

        self.frame_num = 0

        self.bounding_box = None
        self.b_draw_bounding_boxes = False

        self.video_capture = None
        self.current_image = None

    def process_bound_data(self, data):
        sensor_id_str = str(self.sensor.sensor_id)
        return data['bounding_box_positions'][sensor_id_str], data['in_camera_los'][sensor_id_str]

    def initialize_views(self):
        self.view_lock.acquire()
        super(CameraWidget, self).initialize_views()

        self.panel = Label()
        self.panel.pack(fill=BOTH)

        self.view_lock.release()

    def update_widget(self, data):
        bb_pos = None
        bb_in_fov = False
        if self.bounding_box is not None:
            bounding_data = self.bounding_box.get_data()
            bb_pos, bb_in_fov = self.process_bound_data(bounding_data)

        if self.sensor.hdmi_streaming:
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
            self.current_image = self.current_image[start_y:start_y + standalone_height,
                                 start_x:start_x + standalone_width]

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
            self.current_image = self.sensor.get_q_image()

            image = self.current_image
            if self.bounding_box is not None and self.b_draw_bounding_boxes:
                image = self.draw_bounding_boxes(self.current_image, bb_pos, bb_in_fov)

            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(image)
            image = ImageTk.PhotoImage(image)

            # Only update window size when new image size
            if self.master_tk.winfo_width() != len(self.current_image[0]) or self.master_tk.winfo_height() != len(
                    self.current_image):
                window_size = str(len(self.current_image[0])) + 'x' + str(len(self.current_image))
                self.master_tk.geometry(window_size)
            self.panel.configure(image=image)
            self.panel.image = image

            self.frame_num += 1

    def draw_bounding_boxes(self, q, bounding_box_positions, bounding_box_in_fov):
        overlay1 = q.copy()
        overlay2 = q.copy()
        output = q.copy()

        for i in range(0, len(bounding_box_positions)):
            pos0_x = bounding_box_positions[i][0]
            pos0_y = bounding_box_positions[i][1]
            pos1_x = bounding_box_positions[i][2]
            pos1_y = bounding_box_positions[i][3]
            pos2_x = bounding_box_positions[i][4]
            pos2_y = bounding_box_positions[i][5]
            pos3_x = bounding_box_positions[i][6]
            pos3_y = bounding_box_positions[i][7]
            pos4_x = bounding_box_positions[i][8]
            pos4_y = bounding_box_positions[i][9]
            pos5_x = bounding_box_positions[i][10]
            pos5_y = bounding_box_positions[i][11]
            pos6_x = bounding_box_positions[i][12]
            pos6_y = bounding_box_positions[i][13]
            pos7_x = bounding_box_positions[i][14]
            pos7_y = bounding_box_positions[i][15]
            is_in_fov = bounding_box_in_fov[i]

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


class GPSWidget(TkinterSensorUI):
    def __init__(self, sensor):
        super(GPSWidget, self).__init__(sensor)
        self.framing = None
        self.forward_vector = None
        self.world_location = None
        self.speed = None

    def initialize_views(self):
        self.view_lock.acquire()
        super(GPSWidget, self).initialize_views()

        self.string_lat = StringVar()
        self.lat_text_display = Label(self.master_tk, textvariable=self.string_lat)
        self.lat_text_display.pack()

        self.string_lng = StringVar()
        self.lng_text_display = Label(self.master_tk, textvariable=self.string_lng)
        self.lng_text_display.pack()

        self.string_time = StringVar()
        self.time_text_display = Label(self.master_tk, textvariable=self.string_time)
        self.time_text_display.pack()

        self.view_lock.release()

    def update_widget(self, data):
        self.string_lat.set('LAT: {0}'.format(data['lat']))
        self.string_lng.set('LNG: {0}'.format(data['lng']))
        self.string_time.set('TIMESTAMP: {0}'.format(data['time_stamp']))


class IMUWidget(TkinterSensorUI):
    def __init__(self, sensor):
        super(IMUWidget, self).__init__(sensor)
        self.framing = None
        self.acceleration_vector = None
        self.angular_velocity_vector = None
        self.timer = None

    def initialize_views(self):
        self.view_lock.acquire()

        super(IMUWidget, self).initialize_views()

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

    def update_widget(self, data):
        self.string_accel_x.set('ACCEL_X: {0}'.format(data['acceleration_vector'][0]))
        self.string_accel_y.set('ACCEL_Y: {0}'.format(data['acceleration_vector'][1]))
        self.string_accel_z.set('ACCEL_Z: {0}'.format(data['acceleration_vector'][2]))
        self.string_ang_rate_x.set('ANG RATE X: {0}'.format(data['angular_velocity_vector'][0]))
        self.string_ang_rate_y.set('ANG RATE Y: {0}'.format(data['angular_velocity_vector'][1]))
        self.string_ang_rate_z.set('ANG RATE X: {0}'.format(data['angular_velocity_vector'][2]))
        self.string_timer.set('TIMESTAMP: {0}'.format(data['time_stamp']))


class LidarWidget(BaseSensorUI):
    def __init__(self, sensor):
        super(LidarWidget, self).__init__(sensor)

        self.veloview_socket = self.connect()

    def update_widget(self, data):
        for frame in data:
            self.veloview_socket.sendto(frame, (VELOVIEW_IP, VELOVIEW_PORT))

    def connect(self):
        """ Setup connection to the socket listening in Unreal. """
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.veloview_socket = s
            return s

        except Exception as e:
            print('Can send to {0}'.format(str((VELOVIEW_PORT, VELOVIEW_PORT))))
            print("Error {0}".format(e))
            self.veloview_socket = None
            return None


class MultiCameraWidget(CameraWidget):
    def __init__(self, sensor):
        super(MultiCameraWidget, self).__init__(sensor)
        self.camera_ids = sensor.camera_ids

    def process_bound_data(self, data):

        bounding_box_positions = []
        bounding_box_is_in_camera_fov = []
        i = 0
        for camera_id in self.camera_ids:
            single_camera_width = self.sensor.width / len(self.camera_ids)
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

                    bounding_box_positions.append(positions)
                    bounding_box_is_in_camera_fov.append(in_line_of_sight)

            i = i + 1

        return bounding_box_positions, bounding_box_is_in_camera_fov

    def check_in_bounds(self, positions):
        for i in range(0, len(positions)):
            pos_point = positions[i]
            if pos_point < 0 or pos_point > 512:
                return False

        return True


class RPMWidget(TkinterSensorUI):
    def __init__(self, sensor):
        super(RPMWidget, self).__init__(sensor)

    def initialize_views(self):
        self.view_lock.acquire()

        super(RPMWidget, self).initialize_views()

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

    def update_widget(self, data):
        self.string_wheel_number.set('Wheel Number: {0}'.format(data['wheel_number']))
        self.string_wheel_rpm.set('RPM: {0}'.format(data['wheel_rpm']))
        self.string_timestamp.set('Time Stamp: {0}'.format(data['time_stamp']))


class WaypointWidget(MatplotlibSensorUI):
    def __init__(self, sensor):
        super(WaypointWidget, self).__init__(sensor)
        self.previous_points = None
        self.xy_combined = np.column_stack(([0], [0]))

    def initialize_views(self):
        self.view_lock.acquire()
        super(WaypointWidget, self).initialize_views()
        self.map_subplot = self.main_plot.add_subplot(111)
        self.map_plot_handle, = self.map_subplot.plot(0, 0, marker='o', linestyle='None')
        self.map_subplot.set_title("Ground Truth Waypoint Map")
        self.view_lock.release()

    def display(self, x, y):
        self.map_plot_handle.set_xdata(x)
        self.map_plot_handle.set_ydata(y)
        margin = 10
        plt.axis((min(x) - margin, max(x) + margin, min(y) - margin, max(y) + margin))

    def update_widget(self, data):
        self.view_lock.acquire()
        points_by_lane = data['points_by_lane']
        x_combined = []
        y_combined = []
        for points in points_by_lane:
            x_combined = np.append(x_combined, points[:, 0])
            y_combined = np.append(y_combined, points[:, 1])

        self.xy_combined = np.column_stack((x_combined, y_combined))
        self.view_lock.release()

    def update_views(self, frame):
        self.view_lock.acquire()
        self.display(self.xy_combined[:, 0], self.xy_combined[:, 1])
        self.view_lock.release()
        return self.map_subplot


