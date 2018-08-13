__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import datetime
import json
import multiprocessing
import os.path

import os

try:
    import queue
except ImportError:
    import Queue as queue

import logging
import socket
import struct
import threading
import time
import sys,traceback

import Queue as PythonQueue
from multiprocessing import Queue, Process
from monodrive.networking.queues import SingleQueue

from monodrive.networking import messaging
from monodrive.networking.packetizer import Packetizer
from monodrive.transform import Rotation, Transform, Translation

BITS_PER_BYTE = 8.0
BYTE_PER_MBYTE = 1000000.0


#class BaseSensor(multiprocessing.Process):
class BaseSensor(threading.Thread):
    def __init__(self, idx, config, simulator_config, **kwargs):
        super(BaseSensor, self).__init__(**kwargs)
        self.idx = idx
        #self.daemon = True
        #self.q_data = SingleQueue()
        #self.q_display = SingleQueue()
        self.q_data = multiprocessing.Queue()
        self.q_display = multiprocessing.Queue()
        self.rx_pipe, self.tx_pipe = multiprocessing.Pipe(duplex=False)
        self.socket_ready_event = multiprocessing.Event()
        self.message_event = multiprocessing.Event()
        self.sensor_initialized_event = multiprocessing.Event()
        self.config = config
        self.server_ip = simulator_config.server_ip
        self.client_ip = simulator_config.client_ip
        self.type = self.config['type']
        self.sensor_id = self.config['id']
        self.fps = self.config['fps']
        self.drop_frames = True
        self.running = True
        self.start_time = None
        self.number_of_packets = 0
        self.frame_count = 0
        self.receiving_data = False
        self.update_timer = 1
        self.packetizer_process = None
        self.game_time = None
        self.time_stamp = None
        self.socket_udp = config.get('connection_type', None) == "udp"
        self.running_socket = True

        self.listen_port = int(config['listen_port'])
        self.packet_size = int(config["packet_size"])

        if self.socket_udp:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.packet_size * 500)
        else:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            #self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        self.sensor_initialized_event.set()
        


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

    def get_messages(self, block=False):
        messages = []
        try:
            while self.q_data.qsize():
                msg = self.q_data.get(block=False)
                messages.append(msg)
                # If `False`, the program is not blocked. `Queue.Empty` is thrown if 
                # the queue is empty
        except PythonQueue.Empty:
            print("{0} Q_EMPTY".format(self.name))
            msg = "EMPTY"
            messages.append(msg)        
        return messages


    def get_message(self, timeout=1, block=True):
        try:
            data = self.q_data.get(block=False, timeout=timeout)
        except PythonQueue.Empty as e:
            logging.getLogger("sensor").warning("{0}:get_message->{1}".format(self.name, e))
            
        #if "SHUTDOWN" in data:
            #return "SHUTDOWN"
        return data

    def get_display_messages(self, timeout=1, block=True):
        messages = []
        if self.q_display.qsize() == 0:
            messages.append("NO_DATA")
        try:
            while self.q_display.qsize():
                msg = self.q_display.get(block=False)
                messages.append(msg)
                # If `False`, the program is not blocked. `Queue.Empty` is thrown if 
                # the queue is empty
        except PythonQueue.Empty:
            print("{0} Display Q_EMPTY".format(self.name))
            msg = "NO_DATA"
            messages.append(msg)        
        return messages

    def stop(self):
        
        logging.getLogger("sensor").info('*** %s' % self.name)
        #self.sock.stop()
        print("{0} socket is closing".format(self.name))
        try:
            self.running_socket = False
            self.sock.shutdown(socket.SHUT_RDWR)
            self.sock.close()
            self.sock = None
        except Exception as e:
            print("{0} could not close socket error:{1}".format(self.name, e))
        print("{0} socket is closed".format(self.name))
        #self.q_data.put("SHUTDOWN")
        #self.q_display.put("SHUTDOWN")
        time.sleep(1)
        self.running = False  # Will stop UDP, Logging thread, and GUI
        print("{0} self.running is set to false".format(self.name))
        return 

 
    def terminate(self):
        #self.running = False
        #super(BaseSensor, self).stop()
        logging.getLogger("sensor").info('*** %s' % self.name)

    def read(self, length):
        received = 0
        data = b''
        #TODO if recv_buffer = 0 break
        while received < length:
            recv_buffer = self.sock.recv(length - received)
            data += recv_buffer
            received += len(recv_buffer)

        return received, data

    def get_packet(self):

        if not self.running_socket:
            return "SHUTDOWN", None, None

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
                #print("{0} packet received l:{1} t:{2} g:{3}".format(self.name, length, time_stamp, game_time))
                length = length - header_size

                received, packet = self.read(length)

                if len(packet) != length or received != length:
                    print("{0}: incomplete frame received: {1} != {2} (rcv count: {3})".format(
                        self.name, len(packet), length, received))
            else:
                print("incomplete header: {0},{1},{2}".format(header_size, received, packet_header))

        return packet, time_stamp, game_time

    def connect(self):
        tries = 0
        connected = False

        while self.running and not connected:
            if self.socket_udp:
                logging.getLogger("network").debug(
                    'Setting udp listening port on %s for %s' % (self.listen_port, self.name))
                self.sock.bind(('', self.listen_port))
                connected = True
            else:
                logging.getLogger("network").debug(
                    'connecting tcp sensor on %s %s for %s' % (self.server_ip, self.listen_port, self.name))
                try:
                    self.sock.connect((self.server_ip, self.listen_port))
                    connected = True
                except Exception as e:
                    if tries > 5:
                        logging.getLogger("network").error(
                            'Could not connect to %s for %s (%s)' % (self.listen_port, self.name, str(e)))
                        break

                    tries = tries + 1
                    time.sleep(0.25)
                    continue

        if connected:
            self.sock.settimeout(None)
            self.start_time = time.clock()
            self.socket_ready_event.set()
            logging.getLogger("network").debug(
                'connected tcp sensor on %s %s for %s' % (self.server_ip, self.listen_port, self.name))

        return connected

    def run(self):
        if not self.connect():
            return

        while self.running:
            time_stamp = None
            game_time = None
            packet = None
            try:
                packet, time_stamp, game_time = self.get_packet()
            except Exception as e:
                logging.getLogger("network").warning(
                    'Packet exception: %s for %s' % (str(e), self.name))
                print("packet exception: {0}".format(e))
                pass

            #print(self.name, ':', len(packet))
            if packet is not None:
                self.number_of_packets += 1
                self.receiving_data = True
                self.digest_packet(packet, time_stamp, game_time)
            else:
                self.receiving_data = False
                # print("waiting for data...", self.name)
            #TODO make this configurable
            

    # Hook method for digest each packet, when not packetized forward on to digest_frame
    # since each packet is an entire frame. Sensors that send multiple packet for a single data frame
    # need to override this data_ready_event
    def digest_packet(self, packet, time_stamp, game_time):
        self.digest_frame(packet, time_stamp, game_time)

    # Override to manipulate data, see Waypoint and BoundingBox for example
    # Radar and Camera don't need to manipulate the data
    def digest_frame_w_queue(self, frame, time_stamp, game_time):
        #print("digest_frame for {:>16} ts={:>10} gt={:>10}".format(self.name, time_stamp, game_time))
        if "SHUTDOWN" in frame:
            self.q_data.put("SHUTDOWN")
            self.q_display.put("SHUTDOWN")
            return

        self.frame_count += 1
        #self.last_game_time.value = game_time
        #self.log_control_time(self.name, self.last_control_real_time.value)
        if hasattr(self, 'parse_frame'):
            frame = self.parse_frame(frame, time_stamp, game_time)
        self.q_data.put(frame)
        self.q_display.put(frame)
        #if not self.display_process or not self.synchronized_display:
        self.message_event.set()

    def digest_frame_w_pipe(self, frame, time_stamp, game_time):
        print("digest_frame for {:>16} ts={:>10} gt={:>10}".format(self.name, time_stamp, game_time))
    
        self.frame_count += 1
        if hasattr(self, 'parse_frame'):
            frame = self.parse_frame(frame, time_stamp, game_time)
        self.tx_pipe.send(frame)
        self.message_event.set()

    def start_logging(self):
        #self.logging_thread = threading.Thread(target=self._display_logging)
        # self.logging_thread.start()
        pass

    def send_start_stream_command(self, simulator):
        res = simulator.start_sensor_command(self.type, self.listen_port, self.sensor_id,
                                             self.packet_size, self.drop_frames)
        return res

    def send_stop_stream_command(self, simulator):
        
        res = simulator.stop_sensor_command(self.type, self.listen_port, self.sensor_id,
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
        while self.running:
            if not self.receiving_data:
                self.update_timer = 2
                logging.getLogger("network").info("Listening for {0}:{1}".format(self.name, self.listen_port))
            else:
                self.update_timer = 10
                pps, mBps, fps = self.speed()

                logging.getLogger("network").info(
                    "{0}:\t {1} MBps | {2} pps | {3} fps".format(self.name, mBps, pps, fps))
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

    @staticmethod
    def log_control_time(name, previous_control_time):
        dif = time.time() - previous_control_time
        logging.getLogger("sensor").debug('Delay %.5f %s' % (dif, name))


    @property
    def name(self):
        return '{0}_{1}'.format(self.__class__.__name__, self.listen_port)


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