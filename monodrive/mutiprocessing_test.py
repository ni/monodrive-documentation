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

NUMBER_OF_MESSAGES = 10
FPS = float(2)
NUMBER_OF_SENSORS = 4

class Display(multiprocessing.Process):
    def __init__(self, sensors, q_display_list, signal_done_event, ready_event):
        super(Display, self).__init__()
        self.q_display_list = q_display_list
        #self.sensors = sensors
        self.signal_done_event = signal_done_event
        self.stop_event = multiprocessing.Event()
        self.running = True
        self.packet_count = 0
        self.frame_error_count = 0
        self.ready_event = ready_event
        #self.start()

    def run(self):
        self.ready_event.wait()
        print("DISPLAY STARTED")
        start_time = time.time()
        #elapsed_time = 0
        #wait_timeout = 1 / FPS
        #while not self.stop_event.wait(timeout=0 if elapsed_time > wait_timeout else wait_timeout - elapsed_time):
        while not self.signal_done_event.is_set():
            #frame_time = time.time()
            for q in self.q_display_list:
                data = None
                if not self.stop_event.is_set():
                    data = self.get_display_messsage(q)
                if data is not None:
                    self.packet_count += 1

            if self.packet_count >= (NUMBER_OF_SENSORS*NUMBER_OF_MESSAGES - self.frame_error_count):
                pps = NUMBER_OF_MESSAGES/(time.time()- start_time)
                print("display rx received {0} pps: {1}".format(self.packet_count, pps))
                self.signal_done_event.set()
                print("exiting Display: ", self.running)

    def get_display_messsage(self, q):
        try:
            data = q.get(block=True, timeout=1)
            self.frame_error = 0
        except queue.Empty as e:
            self.frame_error_count += 1
            self.frame_error += 1
            print("Empty display q after timeout {0}".format(e))
            if self.frame_error > 10:
                print("signaling done - too many errors")
                self.signal_done_event.set()
            time.sleep(1/FPS/NUMBER_OF_SENSORS)
            data = None
        return data

    def run_backout(self):
        start_time = time.time()
        while self.running:
            for sensor in self.sensors:
                data = None
                data = sensor.get_display_messsage()
                if data:
                    self.packet_count += 1
            if self.packet_count == 4*NUMBER_OF_MESSAGES:
                pps = NUMBER_OF_MESSAGES/(time.time()- START_TIME)
                print("display rx received {0} pps: {1}".format(self.packet_count, pps))
            time.sleep(1/FPS/4)
                    
    def stop(self):
        self.stop_event.set()
        #self.terminate()


class Sensor(multiprocessing.Process):
    def __init__(self, server_ip, port_number, signal_done_event, ready_event):
        #print("starting Sensor on {0}".format(port_number))
        super(Sensor, self).__init__()
        self.rx_pipe,self.tx_pipe = multiprocessing.Pipe()
        #TODO CREATE THIS QUEUE IN
        self.q_data = multiprocessing.Queue()
        #self.q_data = Queue.Queue()
        self.q_display = multiprocessing.Queue()
        #self.q_data = Queue.Queue()
        self.signal_done_event = signal_done_event
        self.ready_event = ready_event
        self.stop_event = multiprocessing.Event()
        self.daemon = True
        self.number_of_packets = 0
        self.server_ip = server_ip
        self.port_number = port_number

        self.running = True # why would we set this here - not actually running?
        self.sock = None  # don't create socket in constructor - wrong process
        #self.start()

    def create_socket(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(1)
        #self.sock.setblocking(0)

    def get_display_messsage(self):
        try:
            data = self.q_display.get(block = True, timeout = 2*(1/FPS))
        except queue.Empty as e:
            print("Empty display q after timeout {0}".format(e))
            data = None
        return data

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

        while self.running and not connected:
            print('connecting tcp sensor on %s %s' % (self.server_ip, self.port_number))
            try:
                self.sock.connect((self.server_ip, self.port_number))
                connected = True
            except Exception as e:
                print('Could not connect to %s for %s - x:%s - (%s)' % (self.server_ip, self.port_number, tries, str(e)))
                if tries > 5:
                    break

                tries = tries + 1
                time.sleep(1)
                continue

        if connected:
            self.sock.settimeout(None)
            self.start_time = time.clock()
            print('connected tcp sensor on %s %s' % (self.server_ip, self.port_number))

        return connected

    def read(self, length):
        received = 0
        data = b''
        while received < length and not self.stop_event.is_set():
            recv_buffer = self.sock.recv(length - received)
            data += recv_buffer
            received += len(recv_buffer)

        return received, data

    def get_packet(self):
        time_stamp = None
        game_time = None
        packet = None

        header_size = 4 + 4 + 4  # length, time_stamp, game_time
        received, packet_header = self.read(header_size)
        if packet_header is not None and received == header_size:
            length, time_stamp, game_time = struct.unpack('=IIf', packet_header)
            #print("{0} packet received len:{1} ts:{2} gt:{3}".format(self.port_number, length, time_stamp, game_time))
            length = length #- header_size

            try:
                received, packet = self.read(length)
            except Exception as e:
                print("Could not read network data {0}".format(e))
                self.stop()

            #print("{0} read packet of length {1}".format(self.name, received))
            if len(packet) != length or received != length:
                print("{0}: incomplete frame received: {1} != {2} (rcv count: {3})".format(
                        self.name, len(packet), length, received))
        else:
            print("incomplete header: {0},{1},{2}".format(header_size, received, packet_header))

        return packet, time_stamp, game_time

    #this will simply wait for stop event - and kill the socket
    def monitor_process_state(self):
        #print("starting thread monitor")
        self.stop_event.wait()
        #print("stop received - shutting down real socket")
        self.shutdown_socket()

    def run(self):
        self.create_socket()
        if not self.connect():
            print("sensor not connected")
            return
        threading.Thread(target=self.monitor_process_state).start()

        

        self.total_delay = 0
        ready_event.wait()
        print("Sensor running on ", self.port_number)
        self.start_time = time.time()
        while not self.stop_event.is_set():
            #self.ready_event.wait()
            time_stamp = None
            game_time = None
            packet = None
            try:
                packet, time_stamp, game_time = self.get_packet()
            except Exception as e:
                print('Packet exception: %s for %s' % (str(e), self.name))
                break

            if packet is not None:
                self.number_of_packets += 1
                data = self.parse_frame(packet, time_stamp, game_time)
                self.forward_frame(data)
                self.total_delay += int(time.time())-time_stamp
            else:
                break

            if(self.number_of_packets == NUMBER_OF_MESSAGES):
                pps = NUMBER_OF_MESSAGES/(time.time()-self.start_time)
                print("sensor {0} received {1} pps: {2}".format(self.port_number, self.number_of_packets, pps))
            #time.sleep(1/FPS/4)
        print("exiting Sensor")

    def shutdown_socket(self):
        if self.sock is not None:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except Exception as e1:
                print("Exception shutting down sensor socket: ", e1)
            finally:
                try:
                    self.sock.close()
                except Exception as e2:
                    print("Exception closing sensor socket: ", e2)
                finally:
                    self.sock = None


    def stop(self):
        self.stop_event.set()


class Image_Message_Server(multiprocessing.Process):
    def __init__(self, port_number, signal_done_event, ready_event):
        super(Image_Message_Server, self).__init__()
        self.image_buffer = []
        self.daemon = True
        self.ip_address = ''
        self.port_number = port_number
        #self.init_image_buffer()
        self.running = True
        self.signal_done_event = signal_done_event
        self.ready_event = ready_event
        self.stop_event = multiprocessing.Event()

        ## dont create socket in constructor - this will create multiple sockets in multiprocessing mode
        self.sock = None
        

        #self.start()

    def create_socket(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((self.ip_address, self.port_number))
        self.sock.settimeout(2)
        self.sock.listen(1)

    def run(self):
        print("image message server started")

        self.create_socket()

        image = cv2.imread(filepath)
        image = image.flatten()
        image = np.append(image,image)
        image = np.append(image,image)
        #image = np.append(image1,image)
        #image = np.append(image,image)
        #image = np.append(image,image)
        length = len(pickle.dumps(image, protocol = -1))
        conn, addr = self.sock.accept()
        conn.settimeout(10)
        print('Connected on %s by %s' % (self.port_number, addr))
        self.start_time = time.time()
        ready_event.set()
        for n in range(NUMBER_OF_MESSAGES):
            #game_time += 1 
            packet_header = struct.pack('=IIf', length, int(time.time()), float(n))
            packet = packet_header + pickle.dumps(image, protocol=-1)
            conn.send(packet)
            #time.sleep(random.random()*.1)
            time.sleep(1/FPS)
            #self.ready_event.set()

        pps = NUMBER_OF_MESSAGES/(time.time()-self.start_time)
        print("{0} packets sent {1} pps = {2}".format(self.name, NUMBER_OF_MESSAGES, pps))
        # time.sleep(2)

        self.stop_event.wait()
        print("closed server connection")
        conn.close()
        self.shutdown_socket()
        print("exiting server")

    def shutdown_socket(self):
        if self.sock is not None:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except Exception as e1:
                print("Exception shutting down server socket: ", e1)
            finally:
                try:
                    self.sock.close()
                except Exception as e2:
                    print("Exception closing server socket: ", e2)
                finally:
                    self.sock = None

    def stop(self):
        self.stop_event.set()

START_TIME = None
if __name__ == "__main__":
    signal_done_event = multiprocessing.Event()
    
    #finish_event = multiprocessing.Event()
    ready_event = multiprocessing.Event()
    ready_event.clear()
    image_server1 = Image_Message_Server(8081, signal_done_event, ready_event)
    image_server2 = Image_Message_Server(8082, signal_done_event, ready_event)
    image_server3 = Image_Message_Server(8083, signal_done_event, ready_event)
    image_server4 = Image_Message_Server(8084, signal_done_event, ready_event)
    servers = [image_server1, image_server2, image_server3, image_server4]

    sensor1 = Sensor('127.0.0.1', 8081, signal_done_event, ready_event)
    sensor2 = Sensor('127.0.0.1', 8082, signal_done_event, ready_event)
    sensor3 = Sensor('127.0.0.1', 8083, signal_done_event, ready_event)
    sensor4 = Sensor('127.0.0.1', 8084, signal_done_event, ready_event)
    
    sensors = [sensor1, sensor2, sensor3, sensor4]

    q_display_list = [sensor1.q_display, sensor2.q_display, sensor3.q_display, sensor4.q_display]

    display = Display(sensors, q_display_list, signal_done_event, ready_event)
    START_TIME = time.time()
    [server.start() for server in servers]
    #time.sleep(1)  # give a sec for servers to start
    [sensor.start() for sensor in sensors]

    display.start()

    signal_done_event.wait(timeout = (30+NUMBER_OF_MESSAGES/FPS))
    #time.sleep(1)

    display.stop()
    display.join()

    #these are stopping it in main process - not sensor/server process
    [sensor.stop() for sensor in sensors]
    [server.stop() for server in servers]

    [server.join() for server in servers]
    [sensor.join() for sensor in sensors]

    print("DONE - exiting main")


