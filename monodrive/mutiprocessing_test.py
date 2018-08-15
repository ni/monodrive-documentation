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
FPS = float(1)
NUMBER_OF_SENSORS = 1
HEADER_SIZE = 12

class Display(multiprocessing.Process):
    def __init__(self, sensors, evt_run, evt_shutdown, evt_display_rx_complete):
        super(Display, self).__init__()
        self.sensors = sensors
        self.evt_run_display = evt_run
        self.evt_shutdown_display = evt_shutdown
        self.evt_display_rx_complete = evt_display_rx_complete
        self.frame_count = None
        self.frame_error = None
        self.frame_error_count = None
        self.start()

    def im_complete(self):
        self.evt_display_rx_complete.set()

    def run(self):
        print("DISPLAY STARTED {0}".format(time.time()))
        start_time = time.time()
        self.frame_count = 0
        self.frame_error = 0
        self.frame_error_count = 0
        total_bytes_received = 0
        self.evt_run_display.wait()
        print("****************START DISPLAYING!!!*****************")

        while self.evt_run_display.is_set():
            #frame_time = time.time()
            for sensor in self.sensors:
                data = None
                if self.evt_run_display.is_set():
                    data = sensor.get_display_messsage()
                    if data:
                        image = len(data['image'])
                        if  image: 
                            total_bytes_received += image + HEADER_SIZE
                            self.frame_count += 1
                        else:
                            print("DISPLAY did not get an image")

            #if self.frame_count % (NUMBER_OF_MESSAGES/NUMBER_OF_SENSORS) == 0:
            print("DISPLAY rx frame count = {0}".format(self.frame_count))

            if self.frame_count >= NUMBER_OF_SENSORS*NUMBER_OF_MESSAGES:
                print("****************DISPLAY DONE!!!*****************")
                pps = self.frame_count / (time.time()- start_time)
                print("DISPLAY rx received {0} pps: {1} total_bytes_received {2}"
                       .format(self.frame_count, pps, total_bytes_received/NUMBER_OF_SENSORS))
                self.im_complete()
        #self.evt_shutdown_display.wait()
        
class Sensor(object):
    def __init__(self, server_ip, port_number, evt_run, evt_shutdown):
        print("starting Sensor on {0}".format(port_number))
        self.q_vehicle = multiprocessing.Queue()
        self.q_display = multiprocessing.Queue()
        self.evt_run_sensor = evt_run  #stop process, clear queue 
        self.evt_shutdown_sensor = evt_shutdown
        self.shutdown_complete = multiprocessing.Event()

        self.server_ip = server_ip
        self.port_number = port_number
        
        #process_args = (self.server_ip, self.port_number, self.q_vehicle, self.q_display, self.evt_run_sensor)
        #self.process = multiprocessing.Process(target=self.sensor_loop, args=process_args)
        self.process = multiprocessing.Process(target=self.sensor_loop)
        self.process.name = "Sensor:" + str(self.port_number)
        self.name = self.process.name

        self.errors_max = 10
        self.number_of_packets = 0
        self.sensor_sock = None  #this is an object used by process don't initialize it here
        self.monitor = None      #used to monitor process to enable shutdown
        self.frame_error = 0
        self.process.start()

    def stop(self):
        self.shutdown_complete.wait()
        try:
            self.monitor.join(timeout = 2)
            self.process.join(timeout = 2)
        except Exception as e:
            print("could not join process {0} -> {1}".format(self.name, e))

    #this will simply wait for sensor to stop running - and kill the socket
    def monitor_process_state(self):
        print("{0} starting thread monitor".format(self.name))
        self.evt_run_sensor.wait()
        print("{0} thread monitor waiting for shutdown".format(self.name))
        self.evt_shutdown_sensor.wait()
        print("{0} stop received - shutting down real socket".format(self.name))
        if self.shutdown_socket():
            print("{0} shutdown complete set".format(self.name))
            self.shutdown_complete.set()

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

    def get_display_messsage(self):
        try:
            data = self.q_display.get(block=True, timeout = 1)
            self.frame_error = 0  #reset frame error counter because we got data from q
        except queue.Empty as e:
            self.frame_error += 1
            print("Empty display q after timeout {0}".format(e))
            if self.frame_error > self.errors_max:
                print("signaling done - too many errors")
            time.sleep(1/FPS)
            data = []
        return data

    def create_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #sock.settimeout(10)
        #sock.setblocking(0)
        return sock
        #self.sock.setblocking(0)

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
        while not connected:
            #print('%s connecting tcp sensor on %s %s' % (self.name, self.server_ip, self.port_number))
            try:
                self.sensor_sock.connect((self.server_ip, self.port_number))
                connected = True
            except Exception as e:
                print('%s Could not connect to %s for %s - x:%s - (%s)' % (self.name, self.server_ip, self.port_number, tries, str(e)))
                if tries > 5:
                    break
                tries = tries + 1
                time.sleep(.2)
                continue

        if connected:
            #self.sensor_sock.settimeout(None)
            print('connected tcp sensor on %s %s' % (self.server_ip, self.port_number))
        return connected

    def read(self, length):
        received = 0
        data = b''
        #while received < length and self.evt_run_sensor.is_set():
        while received < length:
            recv_buffer = self.sensor_sock.recv(length - received)
            if recv_buffer is None:
                raise Exception("error reading from socket")
            data += recv_buffer
            received += len(recv_buffer)
            print self.name + " recv'd:" + str(received)
        return received, data

    def get_packet(self):
        time_stamp = None
        game_time = None
        packet = None

        header_size = 4 + 4 + 4  # length, time_stamp, game_time
        received, packet_header = self.read(header_size)
        if packet_header is not None and received == header_size:
            length, time_stamp, game_time = struct.unpack('=IIf', packet_header)
            print("{0} packet received len:{1} ts:{2} gt:{3}".format(self.name, length, time_stamp, game_time))
            length = length - header_size
            try:
                received, packet = self.read(length)
            except Exception as e:
                print("Could not read network data {0}".format(e))
                #self.stop()
            #print("{0} read packet of length {1}".format(self.name, received))
            if len(packet) != length or received != length:
                print("{0}: incomplete frame received: {1} != {2} (rcv count: {3})".format(
                        self.name, len(packet), length, received))
        else:
            print("incomplete header: {0},{1},{2}".format(header_size, received, packet_header))

        return packet, time_stamp, game_time


    #def sensor_loop(self, q_vehicle, q_display, evt_run_sensor):
    def sensor_loop(self):
        print "%s starting process..." % self.name
        self.sensor_sock = self.create_socket()
        self.start_time = None

        if not self.connect():
            print("{0} could not connect to server at {1}".format(self.name, self.server_ip))

        self.monitor = threading.Thread(target=self.monitor_process_state).start()
        
        self.start_time = time.time()
        print("{0} STARTED at {1}".format(self.name,self.start_time))
        
        #self.evt_run_sensor.wait()
        #TODO need a good why to stop this loop
        while not self.evt_shutdown_sensor.is_set:
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
                

class Image_Message_Server(object):
    def __init__(self, port_number, evt_start, evt_shutdown, evt_server_running):
        super(Image_Message_Server,self).__init__()
        self.ip_address = ''
        self.port_number = port_number

        self.shutdown_complete = multiprocessing.Event()
        self.evt_run_server = evt_start
        self.evt_sever_running = evt_server_running
        #self.server_stop = multiprocessing.Event()  #stop process, clear queue
        self.evt_shutdown = evt_shutdown  #stop process, clear queue, close socket 
        
        self.process = multiprocessing.Process(target=self.server_loop)
        self.process.name = "Server:" + str(self.port_number)
        self.name = self.process.name

        ## dont create socket in constructor - this will create multiple sockets in multiprocessing mode
        self.server_sock = None
        self.process.start()

    def create_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.bind((self.ip_address, self.port_number))
        sock.listen(1)
        return sock

    def create_frame_payload(self):
        image = cv2.imread(filepath)
        image = image.flatten()
        #image = np.append(image, image)
        #image = np.append(image, image)
        return image

    def server_loop(self):
        
        #print("%s image message server started" % self.name)
        #create socket here so it lives in process
        server_sock = self.create_socket()
        threading.Thread(target=self.monitor_process_state, args=(server_sock,)).start()
        #static image payload to emulate camera sensor
        image = self.create_frame_payload()

        data_length = len(pickle.dumps(image, protocol = -1))
        evt_server_running.set()
        print("server running and ready to accept")
        conn, addr = server_sock.accept()
        print('%s connected on %s by %s' % (self.name, self.port_number, addr))
        self.start_time = time.time()
        
        header_length = 12
        packet_length = header_length + data_length
        total_bytes_sent = 0
        
        #self.evt_run_server.wait()  #******GLOBAL evt_run*******

        for n in range(NUMBER_OF_MESSAGES):
            packet_header = struct.pack('=IIf', int(packet_length), int(time.time()), float(n))
            packet = packet_header + pickle.dumps(image, protocol=-1)
            try:
                conn.sendall(packet)
            except Exception as e:
                print("{0} sending exception {1}".format(self.name, e))
                time.sleep(.1)
                continue
            total_bytes_sent += len(packet)
            #time.sleep(2*random.random()/FPS)
            time.sleep(1/FPS)
        pps = NUMBER_OF_MESSAGES/(time.time()-self.start_time)
        print("{0} packets sent {1} pps = {2} total_bytes_sent={3}".format(self.name, NUMBER_OF_MESSAGES, pps, total_bytes_sent))
        
        while self.evt_run_server.is_set(): # while running sleep here because we are done
            time.sleep(.1)
        try:
            conn.close()
        except Exception as e:
            print("problem closing connection: {0}".format(e))

    def shutdown_socket(self, server_sock):
        print("{0} shut_down sockt".format(self.name))
        if server_sock is not None:
            try:
                self.server_sock.shutdown(socket.SHUT_RDWR)
            except Exception as e1:
                print("Exception shutting down server socket: ", e1)
            finally:
                try:
                    server_sock.close()
                except Exception as e2:
                    print("Exception closing server socket: ", e2)
                finally:
                    server_sock = None
                    return True
        return False

    #this will simply wait for stop event - and kill the socket
    def monitor_process_state(self, server_sock):
        print("{0} starting thread monitor".format(self.name))
        self.evt_run_server.wait()
        print("{0} starting thread monitor waiting for shutdown".format(self.name))
        self.evt_shutdown.wait()
        print("stop received - shutting down real socket {0}".format(self.name))
        if self.shutdown_socket(server_sock):
            print("{0} shutdown complete set".format(self.name))           
            

START_TIME = None
if __name__ == "__main__":
    
    #multiprocessing.set_start_method("spawn")
    #Global Events
    evt_run = multiprocessing.Event()      
    evt_shutdown = multiprocessing.Event()  

    evt_server_running = multiprocessing.Event()
    evt_display_rx_complete = multiprocessing.Event()

    starting_port = 8080
    servers = []
    for n in range(NUMBER_OF_SENSORS):
        port = starting_port + n
        image_server_instance = Image_Message_Server(port, evt_run, evt_shutdown, evt_server_running)
        servers.append(image_server_instance)

    evt_server_running.wait()
    server_ip = '127.0.0.1'
    sensors = []
    for n in range(NUMBER_OF_SENSORS):
        port = starting_port + n
        sensor_instance = Sensor(server_ip, port, evt_run, evt_shutdown)
        sensors.append(sensor_instance)


    display = Display(sensors, evt_run, evt_shutdown, evt_display_rx_complete)
    
    #[server.start() for server in servers]
    #[sensor.sensor_start() for sensor in sensors]
    #display.start()
    evt_run.set()
    #Wait until display process has received all frames sent by image server
    evt_display_rx_complete.wait()
    evt_run.clear()
    evt_shutdown.set()
    print "signal done event recieved"
    [sensor.shutdown_complete.wait() for sensor in sensors]
    [server.shutdown_complete.wait() for server in servers]

    display.join()

    [server.join() for server in servers]
    [sensor.join() for sensor in sensors]

    print("DONE - exiting main")


