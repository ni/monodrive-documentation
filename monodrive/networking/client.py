
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

from __future__ import absolute_import
from __future__ import print_function

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
        self.socket = None # if socket == None, means client is not connected
        self.wait_connected = threading.Event()

        # Start a thread to get data from the socket
        receiving_thread = threading.Thread(target=self.__receiving)
        receiving_thread.setDaemon(1)
        receiving_thread.start()

    def connect(self, timeout = 90):
        """ Setup connection to the socket listening in Unreal. """
        if not self.isconnected():
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect(self.endpoint)
                self.socket = s
                print("connected to ", self.endpoint)
            except Exception as e:
                print('Can not connect to {0}'.format(str(self.endpoint)))
                print('Is your game running?')
                print("Error {0}".format(e))
                self.socket = None

    def isconnected(self):
        """ Return if the connection is available. """
        return self.socket != None

    def disconnect(self):
        """ Remove the connection with the client properly. """
        if self.isconnected():
            print("BaseClient, request disconnect from server in {0}".format(
                threading.current_thread().name))

            self.socket.shutdown(socket.SHUT_RD)
            # Because socket is on read in __receiving thread,
            # need to call shutdown to force it to close
            if self.socket: # This may also be set to None in the __receiving thread
                self.socket.close()
                self.socket = None
            time.sleep(0.1) # this is tricky

    def __receiving(self):
        """ Method used within thread to retrieve information from the socket. """
        print("TCPClient start receiver on {0}".format(threading.current_thread().name))
        while 1:
            if self.isconnected():
                try:
                    message = Message()
                    message.read(self.socket)
                    # print("<-- ", message)
                except Exception as e:
                    print(str(e))
                    message = None

                if message is None:
                    print('TCPClient: remote disconnected, no more message')
                    self.socket = None
                    continue

                if self.raw_message_handler:
                    self.raw_message_handler(message) # will block this thread
                    # self.raw_message_handler(message2)
                else:
                    # print('No message handler for raw message {0}'.format(
                    #     message))
                    pass

    def send(self, message):
        """ Return response from Unreal """
        if self.isconnected():
            # print("--> ", message)
            return message.write(self.socket)
        else:
            print('Fail to send message, client is not connected')
            return False


class Client(object):
    """
    Client is the public interface for the Unreal communcation.

    """
    def __init__(self, endpoint, message_handler=None):
        self.message_client = BaseClient(endpoint, self.__raw_message_handler)
        self.message_handler = message_handler
        self.message_id = 0
        self.wait_response = threading.Event()
        self.response = ''

        self.isconnected = self.message_client.isconnected
        self.connect = self.message_client.connect
        self.disconnect = self.message_client.disconnect

        self.queue = Queue()
        self.main_thread = threading.Thread(target=self.worker)
        self.main_thread.setDaemon(1)
        self.main_thread.start()

    def __raw_message_handler(self, raw_message):
        self.response = raw_message
        self.wait_response.set()
        if self.message_handler:
            def do_callback():
                self.message_handler(raw_message)

            self.queue.put(do_callback)
        else:
            # Instead of just dropping this message, give a verbose notice
            # print('No message handler to handle message {0}'.format(raw_message))
            pass

    def worker(self):
        while True:
            task = self.queue.get()
            task()
            self.queue.task_done()

    def request(self, message, timeout=90):
        """ Return a response from Unreal """
        def do_request():
            if not self.message_client.send(message):
                return None

        # request can only be sent in the main thread, do not support multi-thread submitting request together
        if threading.current_thread().name == self.main_thread.name:
            do_request()
        else:
            self.queue.put(do_request)
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
            print('Can not receive a response from server. \
                   timeout after {:0.2f} seconds'.format(timeout))
            return None

    def request_sensor_stream(self, message, timeout=3):
        def do_request():
            if not self.message_client.send(message):
                return None

        # request can only be sent in the main thread, do not support multi-thread submitting request together
        if threading.current_thread().name == self.main_thread.name:
            do_request()
        else:
            self.queue.put(do_request)

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
            print('Can not receive a response from server. \
                           timeout after {:0.2f} seconds'.format(timeout))
            return None
