__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"


import multiprocessing

try:
    import queue as PythonQueue
except ImportError:
    import Queue as PythonQueue

import logging
import socket
import struct
import threading
import time

from multiprocessing import Queue, Process
from monodrive.transform import Rotation, Transform, Translation

BITS_PER_BYTE = 8.0
BYTE_PER_MBYTE = 1000000.0


class BaseSensor(object):
    def __init__(self, idx, config, simulator_config, **kwargs):
        super(BaseSensor, self).__init__(**kwargs)

        # sensor config
        self.idx = idx
        self.config = config
        self.type = self.config['type']
        self.sensor_id = self.config['id']
        self.fps = self.config['fps']
        self.packetizer_process = None
        self.game_time = None
        self.time_stamp = None

        # network config
        self.server_ip = simulator_config.server_ip
        self.client_ip = simulator_config.client_ip
        self.port_number = int(config['listen_port'])
        self.packet_size = int(config["packet_size"])
        self.socket_udp = config.get('connection_type', None) == "udp"
        self.sock = None

        # synchronization
        #self.q_data = SingleQueue()
        #self.q_display = SingleQueue()
        self.q_data = multiprocessing.Queue(maxsize=5)
        self.q_display = multiprocessing.Queue(maxsize=5)
        self.socket_ready_event = multiprocessing.Event()
        self.stop_event = multiprocessing.Event()
        self.process = None
        #self.message_event = multiprocessing.Event()

        # tracking / error handling config
        self.drop_frames = True
        self.start_time = None
        self.frame_count = 0
        self.frame_error = 0
        self.number_of_packets = 0
        self.receiving_data = False
        self.update_timer = 1

    @classmethod
    def get_sensor(cls, sensors):
        for s in sensors:
            if type(s).__name__ == cls.__name__:
                return s
        return None

    def get_frame_size(self):
        return self.packet_size

    def get_frame_count(self):
        return self.q_data.qsize()

    def get_message(self, block=True, timeout=None):
        data = None
        try:
            data = self.q_data.get(block=block, timeout=timeout)
        except PythonQueue.Empty as e:
            logging.getLogger("sensor").warning("{0}:get_message->{1}".format(self.name, e))
        return data

    def get_messages(self, block=True, timeout=None):
        messages = []
        try:
            while self.q_data.qsize():
                msg = self.q_data.get(block=block, timeout=timeout)
                messages.append(msg)
                # If `False`, the program is not blocked. `Queue.Empty` is thrown if 
                # the queue is empty
        except PythonQueue.Empty:
            print("{0} Q_EMPTY".format(self.name))
            msg = "EMPTY"
            messages.append(msg)        
        return messages

    def get_display_message(self, block=True, timeout=None):
        try:
            data = self.q_display.get(block=block, timeout=timeout)
            self.frame_error = 0  # reset frame error counter because we got data from q
        except PythonQueue.Empty as e:
            data = None
            self.frame_error += 1
            print("Empty display_q({0}) after timeout {1}".format(self.name, e))
            # if self.frame_error > self.errors_max:
            #     print("signaling done - too many errors")

        return data

    def get_display_messages(self, block=True, timeout=None):
        messages = []
        try:
            msg = self.q_display.get(block=block, timeout=timeout)
            messages.append(msg)
            while self.q_display.qsize():
                msg = self.q_display.get(block=True, timeout=0)
                messages.append(msg)
                # If `False`, the program is not blocked. `Queue.Empty` is thrown if 
                # the queue is empty
        except PythonQueue.Empty:
            #print("{0} Display Q_EMPTY".format(self.name))
            if len(messages) == 0:
                messages.append("NO_DATA")
        return messages

    def start(self):
        self.process = multiprocessing.Process(target=self.sensor_loop)
        self.process.name = self.name
        self.process.start()

    def stop(self):
        self.stop_event.set()

    def join(self, timeout=5):
        try:
            self.process.join(timeout=timeout)
        except Exception as e:
            print("could not join process {0} -> {1}".format(self.name, e))

    def sensor_loop(self):
        monitor = None
        if self.connect():
            monitor = threading.Thread(target=self.monitor_process_state)
            monitor.start()
        else:
            self.stop()

        while not self.stop_event.is_set():
            #time.sleep(1/self.fps/10)  #yeild to GIL at 10x expected frame rate 
            packet, time_stamp, game_time = self.get_packet()
            if packet is not None:
                self.number_of_packets += 1
                self.receiving_data = True
                self.digest_packet(packet, time_stamp, game_time)
            else:
                self.receiving_data = False

        # we're done just make sure monitor is done done
        if monitor is not None:
            monitor.join(timeout=1)

    def connect(self):
        if self.sock is None:
            self.create_socket()

        tries = 0
        connected = False
        while not connected and tries < 5:
            if self.socket_udp:
                logging.getLogger("network").debug(
                    'Setting udp listening port on %s for %s' % (self.port_number, self.name))
                try:
                    self.sock.bind(('', self.port_number))
                    connected = True
                except Exception as e:
                    print('%s Could not bind udp to %s for %s - x:%s - (%s)' % (
                        self.name, self.server_ip, self.port_number, tries, str(e)))
                    tries += 1
                    time.sleep(.2)
            else:
                logging.getLogger("network").debug(
                    'Connecting tcp sensor on %s %s for %s' % (self.server_ip, self.port_number, self.name))
                try:
                    self.sock.connect((self.server_ip, self.port_number))
                    connected = True
                except Exception as e:
                    tries += 1
                    if tries >= 5:
                        print('%s Could not connect to %s for %s - x:%s - (%s)' % (
                            self.name, self.server_ip, self.port_number, tries, str(e)))
                    time.sleep(.2)

        if connected:
            self.start_time = time.clock()
            self.socket_ready_event.set()
            logging.getLogger("network").debug(
                'connected tcp sensor on %s %s for %s' % (self.server_ip, self.port_number, self.name))

        return connected

    def create_socket(self):
        if self.socket_udp:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.packet_size * 500)
        else:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    #this will simply wait for sensor to stop running - and kill the socket
    def monitor_process_state(self):
        print("{0} thread monitor waiting for shutdown".format(self.name))
        self.stop_event.wait()
        print("{0} stop received - shutting down real socket".format(self.name))
        self.shutdown_socket()

    def shutdown_socket(self):
        print("{0} socket is closing".format(self.name))
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
            self.sock.close()
            self.sock = None
        except Exception as e:
            print("{0} could not close socket error:{1}".format(self.name, e))
        print("{0} socket is closed".format(self.name))

    def wait_until_ready(self, timeout=None):
        return self.socket_ready_event.wait(timeout)

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

                try:
                    received, packet = self.read(length)
                except Exception as e:
                    print("Could not read network data({0}) - {1}".format(self.name, e))
                    self.stop()  # socket is dead - must stop

                if len(packet) != length or received != length:
                    print("{0}: incomplete frame received: {1} != {2} (rcv count: {3})".format(
                        self.name, len(packet), length, received))
            else:
                print("incomplete header: {0},{1},{2}".format(header_size, received, packet_header))

        return packet, time_stamp, game_time

    def read(self, length):
        received = 0
        data = b''

        # print("receiving {0} bytes on {1}".format(length, self.name))
        while received < length:
            recv_buffer = self.sock.recv(length - received)
            if recv_buffer is None:
                raise Exception("error reading from socket")
            data += recv_buffer
            received += len(recv_buffer)
            # print(self.name + " recv'd:" + str(received))

        return received, data

    # Hook method for digest each packet, when not packetized forward on to digest_frame
    # since each packet is an entire frame. Sensors that send multiple packet for a single data frame
    # need to override this data_ready_event
    def digest_packet(self, packet, time_stamp, game_time):
        self.digest_frame(packet, time_stamp, game_time)

    # Override to manipulate data, see Waypoint and BoundingBox for example
    # Radar and Camera don't need to manipulate the data
    def digest_frame(self, frame, time_stamp, game_time):
        # print("digest_frame for {:>16} ts={:>10} gt={:>10}".format(self.name, time_stamp, game_time))
        # if "SHUTDOWN" in frame:
        #     self.q_data.put("SHUTDOWN")
        #     self.q_display.put("SHUTDOWN")
        #     return

        self.frame_count += 1
        # self.last_game_time.value = game_time
        # self.log_control_time(self.name, self.last_control_real_time.value)
        if hasattr(self, 'parse_frame'):
            frame = self.parse_frame(frame, time_stamp, game_time)
        self.q_data.put(frame)
        self.q_display.put(frame)
        # if not self.display_process or not self.synchronized_display:
        # self.message_event.set()

    # def digest_frame_w_pipe(self, frame, time_stamp, game_time):
    #     print("digest_frame for {:>16} ts={:>10} gt={:>10}".format(self.name, time_stamp, game_time))
    #
    #     self.frame_count += 1
    #     if hasattr(self, 'parse_frame'):
    #         frame = self.parse_frame(frame, time_stamp, game_time)
    #     self.tx_pipe.send(frame)
    #     self.message_event.set()

    def start_logging(self):
        # self.logging_thread = threading.Thread(target=self._display_logging)
        # self.logging_thread.start()
        pass

    def send_start_stream_command(self, simulator):
        res = None
        if not self.stop_event.is_set():
            res = simulator.start_sensor_command(self.type, self.port_number, self.sensor_id,
                                                 self.packet_size, self.drop_frames)
            if res is None or not res.is_success:
                logging.getLogger("sensor").error(
                    "Failed start stream command for sensor %s %s" % (self.name, res.error_message))
            else:
                logging.getLogger("sensor").info("Sensor ready %s" % self.name)

        return res

    def send_stop_stream_command(self, simulator):
        
        res = simulator.stop_sensor_command(self.type, self.port_number, self.sensor_id,
                                            self.packet_size, self.drop_frames)
        logging.getLogger("sensor").info("***{0}".format(self.name))                                    
        return res

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

    def _display_logging(self):
        self.start_time = time.clock()
        while not self.stop_event.wait(self.update_timer):
            if not self.receiving_data:
                self.update_timer = 2
                logging.getLogger("network").info("Listening for {0}:{1}".format(self.name, self.port_number))
            else:
                self.update_timer = 10
                pps, mBps, fps = self.speed()

                logging.getLogger("network").info(
                    "{0}:\t {1} MBps | {2} pps | {3} fps".format(self.name, mBps, pps, fps))

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

    @staticmethod
    def log_control_time(name, previous_control_time):
        dif = time.time() - previous_control_time
        logging.getLogger("sensor").debug('Delay %.5f %s' % (dif, name))


    @property
    def name(self):
        return '{0}_{1}'.format(self.__class__.__name__, self.port_number)


'''class BaseSensorPacketized(BaseSensor):
    def __init__(self, idx, config, simulator_config, **kwargs):
        super(BaseSensorPacketized, self).__init__(idx=idx, config=config, simulator_config=simulator_config, **kwargs)
        self.q_network = Queue()
        
    def digest_packet(self, packet, time_stamp, game_time):
        super(BaseSensorPacketized, self).digest_packet(packet, time_stamp, game_time)

    def get_message(self, timeout=1, block=True):
        return super(BaseSensorPacketized, self)..get_message()

    def get_display_message(self, timeout=1, block=True):
        return super(BaseSensorPacketized, self)..get_message()'''