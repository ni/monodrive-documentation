
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import datetime
import json
from matplotlib import pyplot as plt
import matplotlib.patches as mpatches
import multiprocessing
import numpy as np
import os.path
try:
    import queue
except ImportError:
    import Queue as queue

import random
import socket
import struct
import threading
import time

from monodrive.networking import messaging
from monodrive.networking.packetizer import Packetizer
from monodrive.transform import Rotation, Transform, Translation

LOGGING = False
SENSOR_WAITING_LOGGING = False


class SensorManager:
    def __init__(self, vehicle_config, simulator_config):
        self.vehicle_config = vehicle_config
        self.simulator_config = simulator_config
        self.sensor_process_dict = {}
        self.sensor_list = []
        self.render_processes = []
        self.sensor_monitor_thread = None
        self.sensor_gametime_graph_thread = None
        self.process_manager_list = multiprocessing.Manager().list()
        self.sensor_data_ready = multiprocessing.Event()
        self.simulator = None
        self.configure()

    def configure(self):
        window_settings = {}
        if os.path.exists('window_settings.json'):
            try:
                with open('window_settings.json') as data_file:
                    window_settings = json.load(data_file)
            except:
                pass

        for sensor_config in self.vehicle_config.sensors:
            if not sensor_config['sensor_process']:
                continue
            self.add(sensor_config, window_settings)

        if self.vehicle_config.bounding_data_on_radar_graph:
            self.attach_bounding_radar()

        if self.vehicle_config.bounding_data_on_camera:
            self.attach_bounding_camera()

    def add(self, sensor_config, window_settings):
        sensor_type = sensor_config['type']
        _Sensor_Class = self.vehicle_config.get_class(sensor_type)
        sensor_instance = _Sensor_Class(sensor_type, sensor_config, self.simulator_config)
        sensor_instance.set_window_coordinates(window_settings)

        self.sensor_process_dict[sensor_instance.name] = sensor_instance
        self.sensor_list.append(sensor_instance)

    def get_process_list(self):
        _processes = []
        for sensor in self.sensor_process_dict.values():
            _processes.append(sensor)
            if getattr(sensor, 'packetizer_process', None) is not None:
                _processes.append(sensor.packetizer_process)
        return _processes

    def start_all_render_processes(self, sensor_list):
        for sensor in sensor_list:
            if not sensor.display_process:
                continue
            render_process_name = sensor.name + '_Render'
            render_process = Process(target=sensor.rendering_main,
                                     name=render_process_name)
            render_process.start()
            self.render_processes.append(render_process)

    def start(self, simulator):
        [p.start() for p in self.get_process_list()]

        for s in self.sensor_list:
            res = s.send_start_stream_command(simulator)
            if not res.is_success:
                print('Failed stream command for sensor', s.name, res.error_message)
            else:
                pass

        self.start_all_render_processes(self.sensor_list)

        for s in self.sensor_list:
            s.socket_ready_event.wait()

        self.sensor_monitor_thread = threading.Thread(target=SensorManager.monitor_sensors,
                                                      args=(self.sensor_data_ready, self.sensor_list))
        self.sensor_monitor_thread.start()

        self.sensor_gametime_graph_thread = threading.Thread(target=SensorManager.graph_sensors_gametimes,
                                                              args=(self.sensor_data_ready, self.sensor_list))
        # self.sensor_gametime_graph_thread.start()

        # Kicks off simulator for stepping from client
        simulator.request(messaging.EgoControlCommand(0.0, 0.0))

    def stop(self, simulator):
        [s.stop(simulator) for s in self.sensor_list]
        [p.terminate() for p in self.render_processes]

    @staticmethod
    def graph_sensors_gametimes(sensor_data_ready, sensors):
        plot = plt.figure(10)
        game_time_subplot = plot.add_subplot(111)
        game_time_subplot.set_title("Sensor Samples by Game Time")
        game_time_subplot.set_xlabel('Frame Number')
        game_time_subplot.set_ylabel('Game Time(milliseconds)')
        game_time_subplot.grid(visible=True)
        # map_subplot.set_title("Ground Truth Waypoint Map")
        while True:
            sensor_data_ready.wait()
            legend = []
            for s in sensors:
                if len(s.game_times) == 0:
                    continue

                y = np.array(s.game_times)
                x = np.arange(len(y))
                # print('plotting', s.name, len(x), len(y))
                if not hasattr(s, 'game_time_plot_handle'):
                    c = (random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1))
                    s.game_time_plot_handle, = game_time_subplot.plot(x, y, marker='o', linestyle='None', color=c)
                    patch = mpatches.Patch(color=c, label=s.name)
                    legend.append(patch)

                else:
                    s.game_time_plot_handle.set_xdata(x)
                    s.game_time_plot_handle.set_ydata(y)
                    plt.pause(.001)
                    margin = 10
                    plt.axis((min(x), max(x) + margin, min(y) - margin, max(y) + margin))
            if len(legend) > 0:
                plt.legend(handles=legend, bbox_to_anchor=(1.0, 1.0))

    @staticmethod
    def monitor_sensors(sensor_data_ready, sensors):
        last_game_time = None
        while True:
            if SENSOR_WAITING_LOGGING:
                print('monitoring sensors')
            if last_game_time is None:
                for s in sensors:
                    if SENSOR_WAITING_LOGGING:
                        print('waiting on first frame for ', s.name)
                    s.data_ready_event.wait()
                    s.data_ready_event.clear()
                    last_game_time = s.last_game_time.value
            else:
                next_expected_sample_time = min(s.next_expected_sample_time for s in sensors)
                tolerance = 10
                sensors_waiting = [s for s in sensors if
                                   s.is_expecting_frame_at_game_time(next_expected_sample_time, tolerance)]
                for s in sensors_waiting:
                    if SENSOR_WAITING_LOGGING:
                        print('waiting on data for:', s.name)
                    received_data = s.data_ready_event.wait(5.0)

                    if not received_data:
                        s.dropped_frame()

                    s.data_ready_event.clear()
                    if SENSOR_WAITING_LOGGING:
                        print('received:', s.name)

            sensor_data_ready.set()

    def attach_bounding_radar(self):
        bounding = None
        for key in self.sensor_process_dict.keys():
            if 'BoundingBox' in key:
                bounding = self.sensor_process_dict[key]
        for key in self.sensor_process_dict.keys():
            if 'Radar' in key:
                radar = self.sensor_process_dict[key]
                if bounding is not None:
                    bounding.sensors_depending_on_data += 1
                    radar.bounding_box = bounding

    def attach_bounding_camera(self):
        bounding = None
        for key in self.sensor_process_dict.keys():
            if 'BoundingBox' in key:
                bounding = self.sensor_process_dict[key]
        for key in self.sensor_process_dict.keys():
            if 'Camera' in key:
                camera = self.sensor_process_dict[key]
                if camera is not None:
                    bounding.sensors_depending_on_data += 1
                    camera.bounding_box = bounding


BITS_PER_BYTE = 8.0
BYTE_PER_MBYTE = 1000000.0


class BaseSensor(multiprocessing.Process):
    def __init__(self, idx, config, simulator_config, **kwargs):
        super(BaseSensor, self).__init__(**kwargs)
        self.idx = idx
        self.q_display = self.init_display_queue()
        self.q_vehicle = self.init_vehicle_queue()
        self.socket_ready_event = multiprocessing.Event()
        self.data_ready_event = multiprocessing.Event()
        self.config = config
        self.server_ip = simulator_config.server_ip
        self.type = self.config['type']
        self.sensor_id = self.config['id']
        self.display_process = self.config['display_process']
        self.fps = self.config['fps']
        self.drop_frames = True
        self.running = True
        self.start_time = None
        self.number_of_packets = 0
        self.frame_count = 0
        self.receiving_data = False
        self.update_timer = 1
        self.logging_thread = None
        self.packetizer_process = None
        self.game_time = None
        self.time_stamp = None
        self.window_x_position = 0
        self.window_y_position = 0
        self.log_level = config.get('logging', None)
        self.socket_udp = config.get('connection_type', None) == "udp"

        self.listen_port = int(config['listen_port'])
        self.packet_size = int(config["packet_size"])

        if self.socket_udp:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.packet_size * 500)
        else:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.sensors_depending_on_data = 1 # the sensor itself depends on the data.

        # Specifically for monitoring
        manager = multiprocessing.Manager()
        self.variances = manager.list()
        self.game_times = manager.list()
        self.last_game_time = manager.Value('f', 0.0)

        self.ready_event_lock = manager.Lock()
        self.sensors_got_data_count = manager.Value('i', 0)

    @classmethod
    def init_display_queue(cls):
        return SingleQueue()

    @classmethod
    def init_vehicle_queue(cls):
        return SingleQueue()

    @classmethod
    def get_sensor(cls, sensors):
        for s in sensors:
            if type(s).__name__ == cls.__name__:
                return s
        return None

    def get_frame_size(self):
        return self.packet_size

    def get_message(self):
        data = self.q_vehicle.peek()
        for key in data:
            setattr(self, key, data[key])
        return data

    def dropped_frame(self):
        print 'Dropped Frame', self.name

    def stop(self, simulator):
        self.running = False  # Will stop UDP and Logging thread
        self.send_stop_stream_command(simulator)
        self.sock.shutdown(1)
        self.sock.close()
        self.sock = None
        self.terminate()

    def terminate(self):
        super(BaseSensor, self).terminate()
        print('terminate $$$$$$$$$$$$$$$$$$')

    def read(self, length):
        received = 0
        data = b''
        while received < length:
            recv_buffer = self.sock.recv(length - received)
            data += recv_buffer
            received += len(recv_buffer)

        return received, data

    def get_packet(self):
        time_stamp = None
        game_time = None
        packet = None

        if self.socket_udp:
            packet, address = self.sock.recvfrom(self.packet_size)
            if self.packetizer_process is None:
                offset = 8
                time_stamp, game_time = struct.unpack('=If', packet[:offset])
                packet = packet[offset:]
        else:
            header_size = 4 + 4 + 4  # length, time_stamp, game_time
            received, packet_header = self.read(header_size)

            if packet_header is not None and received == header_size:
                length, time_stamp, game_time = struct.unpack('=IIf', packet_header)
                # print("{0} packet received l:{1} t:{2} g:{3}".format(self.name, length, time_stamp, game_time))
                length = length - header_size

                received, packet = self.read(length)

                if len(packet) != length or received != length:
                    print("{0}: incomplete frame received: {1} != {2} (rcv count: {3})".format(
                        self.name, len(packet), length, received))
            else:
                print("incomplete header: {0},{1},{2}".format(header_size, received, packet_header))

        return packet, time_stamp, game_time

    def run(self):
        if LOGGING:
            self.start_logging()

        tries = 0
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        while self.running:
            if self.start_time is None:
                if self.socket_udp:
                    print("setting udp listening port on ", self.listen_port)
                    self.sock.bind(('', self.listen_port))
                else:
                    print("connecting tcp sensor on {0}:{1}...".format(self.server_ip, self.listen_port))
                    try:
                        self.sock.connect((self.server_ip, self.listen_port))
                    except:
                        if tries > 5:
                            print("cound not connect to ", self.listen_port)
                            break

                        tries = tries + 1
                        time.sleep(0.1)
                        continue

                self.sock.settimeout(None)
                self.start_time = time.clock()
                self.socket_ready_event.set()
                print("connected tcp sensor on {0}:{1}".format(self.server_ip, self.listen_port))
            else:
                time_stamp = None
                game_time = None
                packet = None
                try:
                    packet, time_stamp, game_time = self.get_packet()
                except Exception as e:
                    print("packet exception: {0}".format(e))
                    pass

                #print(self.name, ':', len(packet))
                if packet is not None:
                    self.number_of_packets += 1
                    self.receiving_data = True
                    self.digest_packet(packet, time_stamp, game_time)
                    if time_stamp is not None:
                        self.log_timestamp(time_stamp, game_time)
                else:
                    self.receiving_data = False
                    #print("waiting for data...", self.name)

    # Hook method for digest each packet, when not packetized forward on to digest_frame
    # since each packet is an entire frame. BaseSensorPacketized overrides this
    def digest_packet(self, packet, time_stamp, game_time):
        self.digest_frame(packet, time_stamp, game_time)

    # Override to manipulate data, see Waypoint and BoundingBox for example
    # Radar and Camera don't need to manipulate the data
    def digest_frame(self, frame, time_stamp, game_time):
        self.frame_count += 1
        self.last_game_time.value = game_time
        if hasattr(self, 'parse_frame'):
            frame = self.parse_frame(frame, time_stamp, game_time)
        self.q_display.put(frame)
        self.q_vehicle.put(frame)
        if not self.display_process:
            self.data_ready_event.set()

    def start_logging(self):
        self.logging_thread = threading.Thread(target=self._display_logging)
        self.logging_thread.start()

    def send_start_stream_command(self, simulator):
        res = simulator.start_sensor_command(self.type, self.listen_port, self.sensor_id,
                                             self.packet_size, self.drop_frames)
        return res

    def send_stop_stream_command(self, simulator):
        res = simulator.stop_sensor_command(self.type, self.listen_port, self.sensor_id,
                                            self.packet_size, self.drop_frames)
        return res

    def _display_logging(self):
        self.start_time = time.clock()
        while self.running:
            if not self.receiving_data:
                self.update_timer = 2
                print("Listening for {0}:{1}".format(self.__class__.__name__, self.listen_port))
            else:
                self.update_timer = 10
                pps, mBps, fps = self.speed()
                print("{0}:\t {1} MBps | {2} pps | {3} fps".format(self.__class__.__name__, mBps, pps, fps))
            time.sleep(self.update_timer)

    def speed(self):
        try:
            frame_size = self.get_frame_size()
        except NotImplementedError:
            frame_size = self.packet_size
        total_time = time.clock() - self.start_time
        pps = round(self.number_of_packets / total_time)
        mBps = pps * self.packet_size / BYTE_PER_MBYTE
        fps = "{:3.2f}".format(pps * self.packet_size / frame_size)
        return pps, mBps, fps

    def log_timestamp(self, time_stamp, game_time):
        variance = BaseSensor.time_stamp_variance(time_stamp)
        self.variances.append(variance)
        self.game_times.append(game_time)

    def is_expecting_frame_at_game_time(self, game_time, tolerance):
        #dif = int(abs(self.last_game_time.value - game_time)) % int(1 / self.fps * 1000)
        #return dif < tolerance
        return True

    @property
    def next_expected_sample_time(self):
        return self.last_game_time.value + (1 / self.fps * 1000)

    @property
    def name(self):
        return '{0}_{1}'.format(self.__class__.__name__, self.listen_port)

    @staticmethod
    def time_stamp_variance(time_stamp):
        now = datetime.datetime.now()
        seconds_since_midnight = (now - now.replace(hour=0, minute=0, second=0, microsecond=0)).total_seconds() * 1000

        day = datetime.datetime.today().weekday() + 1
        seconds_since_sunday = day * 24 * 60 * 60 * 1000
        client_time_stamp = seconds_since_midnight + seconds_since_sunday
        return client_time_stamp - time_stamp

    def update_sensors_got_data_count(self):
        self.ready_event_lock.acquire()

        self.sensors_got_data_count.value += 1

        if self.sensors_got_data_count.value == self.sensors_depending_on_data:
            self.sensors_got_data_count.value = 0
            self.data_ready_event.set()

        self.ready_event_lock.release()

    def get_transform(self):
        position = self.config.get("location", None)
        rotation = self.config.get("rotation", None)
        if position and rotation:
            return Transform(Translation(position['x'], position['y'], position['z']),
                             Rotation(rotation['pitch'], rotation['yaw'], rotation['roll']))
        elif position:
            return Transform(Translation(position['x'], position['y'], position['z']))
        elif rotation:
            return Transform(Rotation(rotation['pitch'], rotation['yaw'], rotation['roll']))

        return None


class BaseSensorPacketized(BaseSensor):
    def __init__(self, idx, config, simulator_config, **kwargs):
        super(BaseSensorPacketized, self).__init__(idx=idx, config=config, simulator_config=simulator_config, **kwargs)
        if self.socket_udp:
            self.q_network = Queue()
            self.packetizer_process = PacketizerSubProcess(self.digest_frame, self.log_timestamp,
                                                           self.q_network, self.packet_size)
            self.packetizer_process.name = self.name + '_Packetizer'

    def stop(self, simulator):
        super(BaseSensorPacketized, self).stop(simulator)
        if self.packetizer_process:
            self.packetizer_process.stop()

    def digest_packet(self, packet, time_stamp, game_time):
        if self.socket_udp:
            self.q_network.put(packet)
        else:
            super(BaseSensorPacketized, self).digest_packet(packet, time_stamp, game_time)


class PacketizerSubProcess(multiprocessing.Process):
    def __init__(self, digest_frame_callback, log_timestamp_callback, q_network, packet_size):
        super(PacketizerSubProcess, self).__init__()
        self.q_network = q_network
        self.digest_frame_callback = digest_frame_callback
        self.log_timestamp_callback = log_timestamp_callback
        self.packetizer = Packetizer(self)
        self.packetizer.packet_size = packet_size
        self.initial_timeout = None
        self.remaining_timeout = 3

    def run(self):
        while True:
            try:
                data = self.q_network.get(True,
                                          self.initial_timeout if self.packetizer.packet_count == 0 else self.remaining_timeout)
            except queue.Empty:
                self.packetizer.on_udp_frame_error()
            else:
                self.packetizer.on_udp_received(data)

    def stop(self):
        self.terminate()

    def digest_frame_delegate(self, data, time_stamp, game_time):
        self.digest_frame_callback(data, time_stamp, game_time)

    def log_timestamp_delegate(self, time_stamp, game_time):
        self.log_timestamp_callback(time_stamp, game_time)


from multiprocessing import Queue, Process
from monodrive.networking.queues import SingleQueue
from .camera import Camera, MultiCamera
from .imu import IMU
from .lidar import Lidar
from .radar import Radar
from .gps import GPS
from .rpm import RPM
from .waypoint import Waypoint
from .bounding_box import BoundingBox
from .rpm import RPM
