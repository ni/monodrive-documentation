from __future__ import absolute_import
from __future__ import print_function

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"


import logging
import socket
import threading
import time

try:
    from Queue import Queue
except:
    from queue import Queue # for Python 3

from monodrive.networking.messaging import Message


class BaseClient(object):
    """
    BaseClient that communicates to Unreal Engine

    There is corresponding software within Unreal that will accept signals
    from a port using TCP.
    """
    def __init__(self, endpoint, raw_message_handler):
        self.endpoint = endpoint
        self.raw_message_handler = raw_message_handler
        self.b_socket_connnected = False # if socket == None, means client is not connected
        self.wait_connected = threading.Event()
        self.sock = None

        # Start a thread to get data from the socket
        self.receiving_thread = threading.Thread(target=self.__receiving)
        self.receiving_thread.setDaemon(1)
        #self.receiving_thread.start()

    def connect(self, timeout = 1):
        """ Setup connection to the socket listening in Unreal. """
        if not self.isconnected():
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                #adding this option so all ports are closed
                #self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.sock.connect(self.endpoint)
                #self.sock = s
                self.b_socket_connnected = True
                self.receiving_thread.start()
                #self.sock = s
                # logging.getLogger("network").info("connected to %s" % self.endpoint)
            except Exception as e:
                # logging.getLogger("network").error(
                #     'Can not connect to {0} \n Is your game running? \n Error {1}'.format(str(self.endpoint), e))
                self.b_socket_connnected = False

    def isconnected(self):
        """ Return if the connection is available. """
        return self.b_socket_connnected

    def disconnect(self):
        """ Remove the connection with the client properly. """
        if self.isconnected():
            logging.getLogger("network").info("BaseClient, request disconnect from server in {0}".format(
                 threading.current_thread().name))
            self.b_socket_connnected = False

            #time.sleep(1)
            self.sock.shutdown(socket.SHUT_RDWR)
            # Because socket is on read in __receiving thread,
            # need to call shutdown to force it to close
            if self.sock: # This may also be set to None in the __receiving thread
                self.sock.close()
                self.sock = None
            #time.sleep(0.1) # this is tricky
            

    def __receiving(self):
        """ Method used within thread to retrieve information from the socket. """
        # logging.getLogger("network").info("TCPClient start receiver on {0}".format(threading.current_thread().name))
        while self.isconnected():
            try:
                message = Message()
                message.read(self.sock)
                #print("<-- ", message)
            except Exception as e:
                print(str(e))
                logging.getLogger("network").error('Failed to receive message: %s' % str(e))
                message = None

            if message is None:
                # logging.getLogger("network").info('TCPClient: remote disconnected, no more message')
                self.disconnect()
                continue

            if self.raw_message_handler:
                self.raw_message_handler(message) # will block this thread
            else:
                # print('No message handler for raw message {0}'.format(
                #     message))
                pass
                

    def send(self, message):
        """ Return response from Unreal """
        if self.isconnected():
            # print("--> ", message)
            return message.write(self.sock)
        else:
            logging.getLogger("network").error('Fail to send message, client is not connected')
            return False


class Client(object):
    """
    Client is the public interface for the Unreal communcation.

    """
    def __init__(self, endpoint):
        self.message_client = BaseClient(endpoint, self.__raw_message_handler)
        self.message_id = 0
        self.wait_response = threading.Event()
        self.response = ''

        self.isconnected = self.message_client.isconnected
        self.connect = self.message_client.connect
        self.disconnect = self.message_client.disconnect
        self.queue = Queue()

        self.data_ready = threading.Event()
        self.stop_event = threading.Event()
        self.main_thread = threading.Thread(target=self.worker, args=(self,))
        self.main_thread.setDaemon(1)
        self.main_thread.start()
        

    def __raw_message_handler(self, raw_message):
        self.response = raw_message
        self.wait_response.set()

    def stop(self, timeout=2):
        logging.getLogger("network").debug('stopping client')
        self.stop_event.set()
        self.main_thread.join(timeout=timeout)
        logging.getLogger("network").debug('stopped client')

    def worker(self, _client):
        while not self.stop_event.is_set():
            if self.data_ready.wait(.1):
                self.data_ready.clear()
                while not self.queue.empty():
                    task = self.queue.get()
                    task()
                    self.queue.task_done()

    def request(self, message, timeout=5):
        """ Return a response from Unreal """
        def do_request():
            if not self.message_client.send(message):
                return None

        # request can only be sent in the main thread, do not support multi-thread submitting request togethergetting here 2
        if threading.current_thread().name == self.main_thread.name:
            do_request()
        else:
            self.queue.put(do_request)
            self.data_ready.set()
        # Timeout is required
        # see: https://bugs.python.org/issue8844
        self.wait_response.clear() # This is important
        isset = self.wait_response.wait(timeout)    # Returns false when event times out
        self.message_id += 1 # Increment only after the request/response cycle finished

        assert(isset != None) # only python prior to 2.7 will return None
        if isset or self.response:
            r = self.response
            self.response = None
            return r
        else:
            logging.getLogger("network").error('Can not receive a response from server. \
                   timeout after {:0.2f} seconds'.format(timeout))
            return None

    def request_sensor_stream(self, message, timeout=1):
        def do_request():
            if not self.message_client.send(message):
                return None

        # request can only be sent in the main thread, do not support multi-thread submitting request together
        if threading.current_thread().name == self.main_thread.name:
            do_request()
        else:
            self.queue.put(do_request)
            self.data_ready.set()

        sensor_ready = False
        isset = None
        while not sensor_ready:
            self.wait_response.clear()  # This is important
            isset = self.wait_response.wait(timeout)  # Returns false when event times out
            if isset:
                sensor_ready = self.response.is_sensor_ready
                self.message_id += 1  # Increment only after the request/response cycle finished

        assert (isset != None)  # only python prior to 2.7 will return None
        if isset or self.response:
            r = self.response
            self.response = None
            return r
        else:
            logging.getLogger("network").error('Can not receive a response from server. \
                           timeout after {:0.2f} seconds'.format(timeout))
            return None
