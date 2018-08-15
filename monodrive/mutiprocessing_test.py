#!/usr/bin/env python

#For image getting
from os import path
import sys

basepath = path.dirname(__file__)
filepath = path.abspath(path.join(basepath, "ui/Capture.png"))

import multiprocessing
import threading
import sys
is_py2 = sys.version[0] == '2'
if is_py2:
    import Queue as queue
else:
    import queue as queue
import struct
import time
import socket
import pickle

import random
import numpy as np 
import cv2

NUMBER_OF_MESSAGES = 100
FPS = float(60)
NUMBER_OF_SENSORS = 4
HEADER_SIZE = 12
PAYLOAD_MULTIPLIER = 1


class Display(object):
    def __init__(self, sensors, evt_display_rx_complete):
        super(Display, self).__init__()
        self.sensors = sensors

        self.evt_display_rx_complete = evt_display_rx_complete
        self.stop_event = multiprocessing.Event()

        self.frame_count = None
        self.frame_error = None
        self.frame_error_count = None

        self.process = multiprocessing.Process(target=self.display_loop)
        self.process.name = "Display Process"
        self.name = self.process.name
        self.process.start()

    def im_complete(self):
        self.evt_display_rx_complete.set()

    def display_loop(self):
        print("DISPLAY STARTED {0}".format(time.time()))
        start_time = time.time()
        self.frame_count = 0
        self.frame_error = 0
        self.frame_error_count = 0
        total_bytes_received = 0
        print("****************START DISPLAYING!!!*****************")

        # we should use the stop event for throttling our display FPS - we can measure time and account for
        # processing delay if we ant
        while not self.stop_event.wait(1 / FPS):
            #frame_time = time.time()
            for sensor in self.sensors:
                data = None
                if not self.stop_event.is_set():
                    data = sensor.get_display_messsage(True, 0)
                    if data:
                        image = len(data['image'])
                        if  image: 
                            total_bytes_received += image + HEADER_SIZE
                            self.frame_count += 1
                        else:
                            print("DISPLAY did not get an image")

            print("DISPLAY rx frame count = {0}".format(self.frame_count))

            if self.frame_count >= NUMBER_OF_SENSORS*NUMBER_OF_MESSAGES and not self.evt_display_rx_complete.is_set():
                print("****************DISPLAY DONE!!!*****************")
                pps = self.frame_count / (time.time()- start_time)
                print("DISPLAY rx received {0} pps: {1} total_bytes_received {2}"
                       .format(self.frame_count, pps, total_bytes_received/NUMBER_OF_SENSORS))
                self.im_complete()

    def stop(self):
        self.stop_event.set()

    def join(self, timeout=5):
        try:
            self.process.join(timeout=timeout)
        except Exception as e:
            print("could not join process {0} -> {1}".format(self.name, e))


class Sensor(object):
    def __init__(self, server_ip, port_number):
        print("starting Sensor on {0}".format(port_number))
        self.q_vehicle = multiprocessing.Queue()
        self.q_display = multiprocessing.Queue()

        self.server_ip = server_ip
        self.port_number = port_number

        self.errors_max = 10  # this should be a factor of FPS and packet_size
        self.frame_error = 0
        self.number_of_packets = 0
        self.sensor_sock = None  #this is an object used by process don't initialize it here
        self.start_time = None
        self.stop_event = multiprocessing.Event()

        self.process = multiprocessing.Process(target=self.sensor_loop)
        self.process.name = "Sensor:" + str(self.port_number)
        self.name = self.process.name
        self.process.start()

    def stop(self):
        self.stop_event.set()

    def join(self, timeout=5):
        try:
            self.process.join(timeout=timeout)
        except Exception as e:
            print("could not join process {0} -> {1}".format(self.name, e))

    #this will simply wait for sensor to stop running - and kill the socket
    def monitor_process_state(self):
        print("{0} thread monitor waiting for shutdown".format(self.name))
        self.stop_event.wait()
        print("{0} stop received - shutting down real socket".format(self.name))
        self.shutdown_socket()

    def shutdown_socket(self):
        print("{0} shut_down socket".format(self.name))
        if self.sensor_sock is not None:
            try:
                self.sensor_sock.shutdown(socket.SHUT_RDWR)
            except Exception as e1:
                print("Exception shutting down sensor socket: ", e1)
            finally:
                try:
                    self.sensor_sock.close()
                except Exception as e2:
                    print("Exception closing sensor socket: ", e2)
                finally:
                    return True
        
        return False

    def get_display_messsage(self, block=True, timeout=1):
        try:
            data = self.q_display.get(block=block, timeout=timeout)
            self.frame_error = 0  #reset frame error counter because we got data from q
        except queue.Empty as e:
            self.frame_error += 1
            print("Empty display_q({0}) after timeout {1}".format(self.name, e))
            if self.frame_error > self.errors_max:
                print("signaling done - too many errors")
            time.sleep(1/FPS) # do we really want to sleep? if we're using a timeout above - this is like a double sleep
            data = None
        return data

    def create_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #sock.settimeout(10)
        #sock.setblocking(0)
        return sock

    def forward_frame(self, message):
        self.q_display.put(message)

    def parse_frame(self, frame, time_stamp, game_time):
        data_dict = {
            'image': frame,
            'game_time': game_time,
            'time_stamp': time_stamp
        }
        return data_dict

    def connect(self):
        tries = 0
        connected = False
        while not connected and tries < 5:
            #print('%s connecting tcp sensor on %s %s' % (self.name, self.server_ip, self.port_number))
            try:
                self.sensor_sock.connect((self.server_ip, self.port_number))
                connected = True
            except Exception as e:
                print('%s Could not connect to %s for %s - x:%s - (%s)' % (self.name, self.server_ip, self.port_number, tries, str(e)))
                tries = tries + 1
                time.sleep(.2)

        if connected:
            #self.sensor_sock.settimeout(None)
            print('connected tcp sensor on %s %s' % (self.server_ip, self.port_number))
        return connected

    def read(self, length):
        received = 0
        data = b''

        # print("receiving {0} bytes on {1}".format(length, self.name))
        while received < length:
            recv_buffer = self.sensor_sock.recv(length - received)
            if recv_buffer is None:
                raise Exception("error reading from socket")
            data += recv_buffer
            received += len(recv_buffer)
            # print(self.name + " recv'd:" + str(received))

        return received, data

    def get_packet(self):
        time_stamp = None
        game_time = None
        packet = None

        header_size = 4 + 4 + 4  # length, time_stamp, game_time
        received, packet_header = self.read(header_size)
        if packet_header is not None and received == header_size:
            length, time_stamp, game_time = struct.unpack('=IIf', packet_header)
            # print("{0} packet received len:{1} ts:{2} gt:{3}".format(self.name, length, time_stamp, game_time))
            length = length - header_size
            try:
                received, packet = self.read(length)
            except Exception as e:
                print("Could not read network data({0}) - {1}".format(self.name, e))
                self.stop()  # socket is dead - must stop
            # print("{0} read packet of length {1}".format(self.name, received))
            if len(packet) != length or received != length:
                print("{0}: incomplete frame received: {1} != {2} (rcv count: {3})".format(
                        self.name, len(packet), length, received))
        else:
            print("incomplete header: {0},{1},{2}".format(header_size, received, packet_header))

        return packet, time_stamp, game_time

    def sensor_loop(self):
        print("%s starting process..." % self.name)
        self.sensor_sock = self.create_socket()

        if self.connect():

            monitor = threading.Thread(target=self.monitor_process_state)
            monitor.start()

            self.start_time = time.time()
            print("{0} STARTED at {1}".format(self.name,self.start_time))

            while not self.stop_event.is_set():
                time_stamp = None
                game_time = None
                packet = None
                try:
                    packet, time_stamp, game_time = self.get_packet()
                except Exception as e:
                    print('%s Packet exception: %s' % (self.name, str(e)))
                    time.sleep(.1)
                    continue

                if packet is not None:
                    self.number_of_packets += 1
                    data = self.parse_frame(packet, time_stamp, game_time)
                    self.forward_frame(data)
                else:
                    time.sleep(.1)
                    continue

                if self.number_of_packets % NUMBER_OF_MESSAGES == 0:
                    pps = NUMBER_OF_MESSAGES/(time.time()-self.start_time)
                    print("sensor {0} received {1} pps: {2}".format(self.port_number, self.number_of_packets, pps))

            #we're done just make sure monitor is done done
            monitor.join(timeout=1)

        else:  # not connected
            print("{0} could not connect to server at {1}".format(self.name, self.server_ip))


class Image_Message_Server(object):
    def __init__(self, port_number):
        super(Image_Message_Server,self).__init__()
        self.ip_address = ''
        self.port_number = port_number

        self.server_sock = None  # this is an in process variable and will be created when the process starts
        self.stop_event = multiprocessing.Event()

        self.process = multiprocessing.Process(target=self.server_loop)
        self.process.name = "Server:" + str(self.port_number)
        self.name = self.process.name
        self.process.start()

    def create_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind((self.ip_address, self.port_number))
        sock.listen(1)
        return sock

    def create_frame_payload(self):
        image = cv2.imread(filepath)
        payload = image.flatten()
        if PAYLOAD_MULTIPLIER > 1:
            frame = image.flatten()
            for i in range(PAYLOAD_MULTIPLIER - 1):
                payload = np.append(payload, frame)
        print("payload size: %sMB's" % (len(payload) / (1024.0*1024.0)))
        return payload

    def server_loop(self):
        
        #print("%s image message server started" % self.name)
        self.server_sock = self.create_socket()
        monitor = threading.Thread(target=self.monitor_process_state)
        monitor.start()

        #static image payload to emulate camera sensor
        image = self.create_frame_payload()

        data_packet = pickle.dumps(image, protocol = -1)
        data_length = len(data_packet)

        print("server running and ready to accept on ", self.port_number)
        conn, addr = self.server_sock.accept()
        print('%s connected on %s by %s' % (self.name, self.port_number, addr))
        self.start_time = time.time()
        
        header_length = 12
        packet_length = header_length + data_length
        total_bytes_sent = 0

        for n in range(NUMBER_OF_MESSAGES):
            packet_header = struct.pack('=IIf', int(packet_length), int(time.time()), float(n))
            packet = packet_header + data_packet
            try:
                # print("sending {0} bytes on {1}".format(packet_length, self.port_number))
                conn.sendall(packet)
            except Exception as e:
                print("{0} sending exception {1}".format(self.name, e))
                time.sleep(.1)
                continue

            total_bytes_sent += len(packet)
            time.sleep(1/FPS)  # artificial delay
            if self.stop_event.is_set():
                break

        pps = NUMBER_OF_MESSAGES/(time.time()-self.start_time)
        print("{0} packets sent {1} pps = {2} total_bytes_sent={3}".format(self.name, NUMBER_OF_MESSAGES, pps, total_bytes_sent))

        self.stop_event.wait()  # if we finish early - wait until we should shutdown?

        try:
            conn.close()
        except Exception as e:
            print("problem closing connection: {0}".format(e))

        monitor.join(timeout=1)

    def shutdown_socket(self):
        print("{0} shut_down socket".format(self.name))
        if self.server_sock is not None:
            try:
                self.server_sock.shutdown(socket.SHUT_RDWR)
            except Exception as e1:
                print("Exception shutting down server socket: ", e1)
            finally:
                try:
                    self.server_sock.close()
                except Exception as e2:
                    print("Exception closing server socket: ", e2)
                finally:
                    self.server_sock = None
                    return True
        return False

    #this will simply wait for stop event - and kill the socket
    def monitor_process_state(self):
        print("{0} starting server thread monitor waiting for shutdown".format(self.name))
        self.stop_event.wait()
        print("server stop received - shutting down socket {0}".format(self.name))
        if self.shutdown_socket():
            print("{0} shutdown complete set".format(self.name))

    def stop(self):
        self.stop_event.set()

    def join(self, timeout=5):
        try:
            self.process.join(timeout=timeout)
        except Exception as e:
            print("could not join process {0} -> {1}".format(self.name, e))
            

START_TIME = None
if __name__ == "__main__":

    # this is an artificial event - when the display thinks we're done it will tell us to shutdown
    evt_display_rx_complete = multiprocessing.Event()

    starting_port = 8080
    servers = []
    for n in range(NUMBER_OF_SENSORS):
        port = starting_port + n
        image_server_instance = Image_Message_Server(port)
        servers.append(image_server_instance)

    server_ip = '127.0.0.1'
    sensors = []
    for n in range(NUMBER_OF_SENSORS):
        port = starting_port + n
        sensor_instance = Sensor(server_ip, port)
        sensors.append(sensor_instance)

    display = Display(sensors, evt_display_rx_complete)

    evt_display_rx_complete.wait()
    print("signal done event received")

    # tell everything to stop
    display.stop()
    [sensor.stop() for sensor in sensors]
    [server.stop() for server in servers]

    # wait for everything to stop
    display.join()
    [server.join() for server in servers]
    [sensor.join() for sensor in sensors]

    print("DONE - exiting main")


