#!/usr/bin/env python

#For image getting
from os import path
import sys

basepath = path.dirname(__file__)
filepath = path.abspath(path.join(basepath, "ui/Capture.png"))

import multiprocessing
import threading
import Queue
import struct
import time
import socket
import pickle

import random
import numpy as np 
import cv2

NUMBER_OF_MESSAGES = 100
FPS = float(30)

class Display(multiprocessing.Process):
    def __init__(self, sensors,q_display_list, stop_event):
        super(Display, self).__init__()
        self.q_display_list = q_display_list
        #self.sensors = sensors
        self.stop_event = stop_event
        self.running = True
        self.packet_count = 0
        self.frame_error = 0
        #self.start()

    def run(self):
        start_time = time.time()
        while self.running:
            for q in self.q_display_list:
                data = None
                data = self.get_display_messsage(q)
                if data != None:
                    self.packet_count += 1

            if self.packet_count == 4*NUMBER_OF_MESSAGES:
                pps = NUMBER_OF_MESSAGES/(time.time()- start_time)
                print("display rx received {0} pps: {1}".format(self.packet_count, pps))
                self.stop_event.set()
            #time.sleep(1/FPS/4)
                   

    def get_display_messsage(self, q):
        try:
            data = q.get(block = True, timeout = 2*(1/FPS))
            self.frame_error = 0
        except Queue.Empty as e:
            self.frame_error += 1
            print("Empty display q after timeout {0}".format(e))
            if self.stop_event.is_set() or self.frame_error > 40:
                self.stop()
            time.sleep(1)
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
        self.running = False
        #self.terminate()


class Sensor(multiprocessing.Process):
    def __init__(self, server_ip, port_number, ready_event):
        #print("starting Sensor on {0}".format(port_number))
        super(Sensor, self).__init__()
        self.rx_pipe,self.tx_pipe = multiprocessing.Pipe()
        #TODO CREATE THIS QUEUE IN
        self.q_data = multiprocessing.Queue()
        #self.q_data = Queue.Queue()
        self.q_display = multiprocessing.Queue()
        #self.q_data = Queue.Queue()

        self.ready_event = ready_event
        self.daemon = True
        self.number_of_packets = 0
        self.server_ip = server_ip
        self.port_number = port_number
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(10)
        self.sock.setblocking(0)
        self.running = True
        #self.start()

    def get_display_messsage(self):
        try:
            data = self.q_display.get(block = True, timeout = 2*(1/FPS))
        except Queue.Empty as e:
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
                if tries > 5:
                    print('Could not connect to %s for %s (%s)' % (self.server_ip, self.port_number, str(e)))
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
        while received < length and not self.ready_event.is_set():
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



    def run(self):
        if not self.connect():
            print("sensor not connected")
            return
        self.total_delay = 0
        self.start_time = time.time()
        while not self.ready_event.is_set():
            #self.ready_event.wait()
            time_stamp = None
            game_time = None
            packet = None
            try:
                packet, time_stamp, game_time = self.get_packet()
            except Exception as e:
                print('Packet exception: %s for %s' % (str(e), self.name))
                pass

            if packet is not None:
                self.number_of_packets += 1
                data = self.parse_frame(packet, time_stamp, game_time)
                self.forward_frame(data)
                self.total_delay += int(time.time())-time_stamp
            if(self.number_of_packets == NUMBER_OF_MESSAGES):
                pps = NUMBER_OF_MESSAGES/(time.time()-START_TIME)
                print("sensor {0} received {1} pps: {2}".format(self.port_number, self.number_of_packets, pps))
            #time.sleep(1/FPS/4)

    def stop(self):
        #self.q_display.put("KILL")
        self.running = False
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()
        self.sock.close()
        self.sock = None


class Image_Message_Server(multiprocessing.Process):
    def __init__(self, port_number, ready_event):
        super(Image_Message_Server, self).__init__()
        self.image_buffer = []
        self.daemon = True
        self.ip_address = ''
        self.port_number = port_number
        #self.init_image_buffer()
        self.running = True
        self.ready_event = ready_event
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(2)
        self.sock.bind((self.ip_address, self.port_number))
        #self.start()

    def run(self):
        print("image message server started")
        
        image = cv2.imread(filepath)
        #image = image.flatten()
        image = np.append(image,image)
        image = np.append(image,image)
        length = len(pickle.dumps(image, protocol = -1))
        self.sock.listen(1)
        conn, addr = self.sock.accept()
        conn.settimeout(10)
        print 'Connect by', addr
        self.start_time = time.time()
        for n in range(NUMBER_OF_MESSAGES):
            #game_time += 1 
            packet_header = struct.pack('=IIf', length, int(time.time()), float(n))
            packet = packet_header + pickle.dumps(image, protocol=-1)
            conn.send(packet)
            #time.sleep(random.random()*.1)
            time.sleep(1/FPS)
            #self.ready_event.set()
        
        pps = NUMBER_OF_MESSAGES/(time.time()-START_TIME)
        print("{0} packets sent {1} pps = {2}".format(self.name, NUMBER_OF_MESSAGES, pps))
        time.sleep(2)
        self.ready_event.wait()
        conn.close()
    
    def stop(self):
        self.running = False
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()
        self.sock = None
START_TIME = None
if __name__ == "__main__":
    connect_ready_event = multiprocessing.Event()
    #finish_event = multiprocessing.Event()
    
    image_server1 = Image_Message_Server(8081,connect_ready_event)
    image_server2 = Image_Message_Server(8082,connect_ready_event)
    image_server3 = Image_Message_Server(8083,connect_ready_event)
    image_server4 = Image_Message_Server(8084,connect_ready_event)
    servers = [image_server1, image_server2, image_server3, image_server4]

    sensor1 = Sensor('127.0.0.1', 8081, connect_ready_event) 
    sensor2 = Sensor('127.0.0.1', 8082, connect_ready_event)
    sensor3 = Sensor('127.0.0.1', 8083, connect_ready_event) 
    sensor4 = Sensor('127.0.0.1', 8084, connect_ready_event)  
    
    sensors = [sensor1, sensor2, sensor3, sensor4]

    q_display_list = [sensor1.q_display, sensor2.q_display, sensor3.q_display, sensor4.q_display]

    display = Display(sensors, q_display_list, connect_ready_event)
    START_TIME = time.time()
    [server.start() for server in servers]
    [sensor.start() for sensor in sensors]
    display.start()

    connect_ready_event.wait(timeout = 30)
    time.sleep(1)

    display.stop()
    
    [sensor.stop() for sensor in sensors]
    [server.stop() for server in servers]
    

    [server.join() for server in servers]
    [sensor.join() for sensor in sensors]


