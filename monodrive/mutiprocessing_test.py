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

NUMBER_OF_MESSAGES = 300
FPS = float(30)
NUMBER_OF_SENSORS = 4
MAX_FRAME_SIZE = 1*1024
HEADER_SIZE = 12

class Display(multiprocessing.Process):
    def __init__(self, sensors, q_display_list, signal_done_event, start_event):
        super(Display, self).__init__()
        self.q_display_list = q_display_list
        self.sensors = sensors
        self.signal_done_event = signal_done_event
        self.stop_display_event = multiprocessing.Event()
        self.frame_count = None
        self.frame_error = None
        self.frame_error_count = None
        self.start_event = start_event
        #self.start()


    def run(self):
        self.start_event.wait()
        print("DISPLAY STARTED {0}".format(time.time()))
        start_time = time.time()
        self.frame_count = 0
        self.frame_error = 0
        self.frame_error_count = 0
        total_bytes_received = 0
        print("****************START DISPLAYING!!!*****************")
        while not self.stop_display_event.is_set():
            #frame_time = time.time()
            for q in self.q_display_list:
                data = None
                if not self.stop_display_event.is_set():
                    data = self.get_display_messsage(q)
                    if data:
                        image = len(data['image'])
                        if  image: 
                            total_bytes_received += image + HEADER_SIZE
                            self.frame_count += 1
                        else:
                            print("DISPLAY did not get an image")

            '''while not self.stop_display_event.is_set():
            #frame_time = time.time()
            for sensor in self.sensors:
                data = None
                if not self.stop_display_event.is_set():
                    data = sensor.get_display_messsage()
                    if data:
                        image = len(data['image'])
                        if  image: 
                            total_bytes_received += image + HEADER_SIZE
                            self.frame_count += 1
                        else:
                            print("DISPLAY did not get an image")'''

            if self.frame_count % (NUMBER_OF_MESSAGES/NUMBER_OF_SENSORS) == 0:
                print("DISPLAY rx frame count = {0}".format(self.frame_count))

            if self.frame_count >= NUMBER_OF_SENSORS*NUMBER_OF_MESSAGES:
                print("****************DISPLAY DONE!!!*****************")
                pps = self.frame_count / (time.time()- start_time)
                print("DISPLAY rx received {0} pps: {1} total_bytes_received {2}"
                       .format(self.frame_count, pps, total_bytes_received/NUMBER_OF_SENSORS))
                self.stop()
                #print("exiting Display")
 
    def stop(self):
        self.stop_display_event.set()
        self.signal_done_event.set()
        

    def get_display_messsage(self, q):
        try:
            data = q.get(block=True, timeout= .5)
            self.frame_error = 0  #reset frame error counter because we got data from q
        except queue.Empty as e:
            self.frame_error_count += 1
            self.frame_error += 1
            print("Empty display q after timeout {0}".format(e))
            if self.frame_error > NUMBER_OF_SENSORS * 4:
                print("signaling done - too many errors")
                #self.stop()
                self.stop_display_event.set()
                self.signal_done_event.set()
            time.sleep(1/FPS/NUMBER_OF_SENSORS)
            data = []
        return data


class Sensor(multiprocessing.Process):
    def __init__(self, server_ip, port_number, signal_done_event, start_event):
        #print("starting Sensor on {0}".format(port_number))
        super(Sensor, self).__init__()
        #self.rx_pipe,self.tx_pipe = multiprocessing.Pipe()
        #TODO CREATE THIS QUEUE IN
        self.q_data = multiprocessing.Queue()
        self.q_display = multiprocessing.Queue()
        
        self.start_event = start_event
        self.signal_done_event = signal_done_event
        self.stop_sensor_event = multiprocessing.Event()
        #self.stop_sensor_socket = multiprocessing.Event()

        self.number_of_packets = 0
        self.server_ip = server_ip
        self.port_number = port_number
        self.sock = None  
        self.frame_error = 0

    def get_display_messsage(self):
        try:
            data = self.q_display.get(block=True, timeout= 1/FPS)
            self.frame_error = 0  #reset frame error counter because we got data from q
        except queue.Empty as e:
            self.frame_error += 1
            print("Empty display q after timeout {0}".format(e))
            if self.frame_error > 4:
                print("signaling done - too many errors")
                #self.stop()
                #self.stop_display_event.set()
                self.signal_done_event.set()
            time.sleep(1/FPS)
            data = []
        return data

    def create_socket(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(1)
        #self.sock.setblocking(0)

    '''def get_display_messsage(self):
        try:
            data = self.q_display.get(block = True, timeout = 2*(1/FPS))
        except queue.Empty as e:
            print("Empty display q after timeout {0}".format(e))
            data = None
        return data'''

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

        while not self.stop_sensor_event.is_set() and not connected:
            #print('%s connecting tcp sensor on %s %s' % (self.name, self.server_ip, self.port_number))
            try:
                self.sock.connect((self.server_ip, self.port_number))
                connected = True
            except Exception as e:
                print('%s Could not connect to %s for %s - x:%s - (%s)' % (self.name, self.server_ip, self.port_number, tries, str(e)))
                if tries > 5:
                    break

                tries = tries + 1
                time.sleep(1)
                continue

        if connected:
            self.sock.settimeout(None)
            print('connected tcp sensor on %s %s' % (self.server_ip, self.port_number))

        return connected

    def read(self, length):
        received = 0
        data = b''
        while received < length and not self.stop_sensor_event.is_set():
            recv_buffer = self.sock.recv(length - received)
            if recv_buffer is None:
                raise Exception("error reading from socket")
            data += recv_buffer
            received += len(recv_buffer)
            #print self.name + " recv'd:" + str(received)

        return received, data

    def get_packet(self):
        time_stamp = None
        game_time = None
        packet = None

        header_size = 4 + 4 + 4  # length, time_stamp, game_time
        received, packet_header = self.read(header_size)
        if packet_header is not None and received == header_size:
            length, time_stamp, game_time = struct.unpack('=IIf', packet_header)
            #print("{0} packet received len:{1} ts:{2} gt:{3}".format(self.name, length, time_stamp, game_time))
            length = length - header_size

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
        #print("{0} starting thread monitor".format(self.name))
        self.stop_sensor_event.wait()
        #print("{0} stop received - shutting down real socket".format(self.name))
        self.shutdown_socket()

    def run(self):
        self.create_socket()
        if not self.connect():
            print("sensor not connected")
            return
        threading.Thread(target=self.monitor_process_state).start()
        self.total_delay = 0
        self.start_event.wait()
        self.start_time = time.time()
        print("{0} STARTED at {1}".format(self.name,self.start_time))
        
        while not self.stop_sensor_event.is_set():
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
                self.signal_done_event.wait()
                print("sensor {0} received {1} pps: {2}".format(self.port_number, self.number_of_packets, pps))
                self.stop_sensor_event.set()
            
        #print("exiting Sensor")

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
        self.stop_sensor_event.set()
        #self.stop_sensor_socket.wait()


class Image_Message_Server(multiprocessing.Process):
    def __init__(self, port_number, signal_done_event, start_event):
        super(Image_Message_Server, self).__init__()
        self.image_buffer = []
        #self.daemon = True
        self.ip_address = ''
        self.port_number = port_number

        self.signal_done_event = signal_done_event
        self.start_event = start_event
        self.stop_server_event = multiprocessing.Event()

        ## dont create socket in constructor - this will create multiple sockets in multiprocessing mode
        self.sock = None


    def create_socket(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((self.ip_address, self.port_number))
        self.sock.settimeout(2)
        self.sock.listen(1)

    #this will simply wait for stop event - and kill the socket
    def monitor_process_state(self):
        #print("starting thread monitor {0}".format(self.name))
        self.stop_server_event.wait()
        #print("stop received - shutting down real socket {0}".format(self.name))
        self.shutdown_socket()

    def create_frame_payload(self):
        image = cv2.imread(filepath)
        image = image.flatten()
        image = np.append(image, image)
        image = np.append(image, image)
        return image

    def run(self):
        #print("%s image message server started" % self.name)

        self.create_socket()
        threading.Thread(target=self.monitor_process_state).start()

        #static image payload to emulate camera sensor
        image = self.create_frame_payload()

        data_length = len(pickle.dumps(image, protocol = -1))
        conn, addr = self.sock.accept()
        #print('%s connected on %s by %s' % (self.name, self.port_number, addr))
        self.start_time = time.time()
        self.start_event.set()
        header_length = 12
        packet_length = header_length + data_length
        total_bytes_sent = 0
        for n in range(NUMBER_OF_MESSAGES):
            packet_header = struct.pack('=IIf', int(packet_length), int(time.time()), float(n))
            packet = packet_header + pickle.dumps(image, protocol=-1)
            
            conn.sendall(packet)
            total_bytes_sent += len(packet)
            ''' or instead of sendall
            while bytes_sent < packet_length:
                idx = min(bytes_sent + MAX_FRAME_SIZE, packet_length - bytes_sent)
                data = packet[bytes_sent:bytes_sent+idx]
                total_bytes_sent += len(data)
                n = conn.send(data) 
                if n > 0:
                    bytes_sent = bytes_sent + n
                else:
                    print("Could not send bytes")
                    time.sleep(1/FPS)'''
            #time.sleep(2*random.random()/FPS)
            time.sleep(1/FPS)

        self.signal_done_event.wait()
        pps = NUMBER_OF_MESSAGES/(time.time()-self.start_time)
        print("{0} packets sent {1} pps = {2} total_bytes_sent={3}".format(self.name, NUMBER_OF_MESSAGES, pps, total_bytes_sent))

        try:
            conn.close()
            self.stop_server_event.set()
        except Exception as e:
            print("problem closing connection: {0}".format(e))

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

START_TIME = None
if __name__ == "__main__":
    
    signal_done_event = multiprocessing.Event()
    start_event = multiprocessing.Event()
    stopped_event = multiprocessing.Event()

    image_server1 = Image_Message_Server(8081, signal_done_event, start_event)
    image_server2 = Image_Message_Server(8082, signal_done_event, start_event)
    image_server3 = Image_Message_Server(8083, signal_done_event, start_event)
    image_server4 = Image_Message_Server(8084, signal_done_event, start_event)
    servers = [image_server1, image_server2, image_server3, image_server4]

    sensor1 = Sensor('127.0.0.1', 8081, signal_done_event, start_event)
    sensor2 = Sensor('127.0.0.1', 8082, signal_done_event, start_event)
    sensor3 = Sensor('127.0.0.1', 8083, signal_done_event, start_event)
    sensor4 = Sensor('127.0.0.1', 8084, signal_done_event, start_event)
    
    sensors = [sensor1, sensor2, sensor3, sensor4]

    q_display_list = [sensor1.q_display, sensor2.q_display, sensor3.q_display, sensor4.q_display]

    display = Display(sensors, q_display_list, signal_done_event, start_event)
    display.start()

    [server.start() for server in servers]
    [sensor.start() for sensor in sensors]

    #Wait until display process has received all frames sent by image server
    signal_done_event.wait()
    print "signal done event recieved"
    #TODO Probably need a event wait here after all processes finish shutdown
    #time.sleep(1)

    display.join()

    [server.join() for server in servers]
    [sensor.join() for sensor in sensors]

    print("DONE - exiting main")


