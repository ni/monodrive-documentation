
from __future__ import absolute_import
from __future__ import print_function

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import json
import struct
import umsgpack

from monodrive.constants import *


class Message(object):
    """
    Base Message object that handles socket IO.

    A Client instance with an open connection to Unreal is required
    for a message object to successfully communicate.
    """

    def __init__(self, cls=u'', data=None):
        self.message_class = cls
        self.status = 0
        self.messages = []
        self.data = data

    def to_json(self):
        return {
            u"class": self.message_class,
            u"status": self.status,
            u"messages": self.messages,
            u"data": self.data
        }

    @property
    def is_success(self):
        return self.data['success']

    @property
    def is_sensor_ready(self):
        return self.data['sensor_ready'] if 'sensor_ready' in self.data else False

    @property
    def error_message(self):
        return self.data['error']

    def __str__(self):
        return json.dumps(self.to_json())

    def read(self, socket):
        """ Read JSON from TCP data from the Unreal Server. """
        rbufsize = 0
        rfile = socket.makefile('rb', rbufsize)
        raw_payload_size = rfile.read(8)
        data = struct.unpack('!II', raw_payload_size)
        magic = data[0]
        length = data[1]

        if magic == RESPONSE_HEADER and length > 8:
            data = umsgpack.unpack(rfile)
            self.message_class = data['class']
            self.status = data['status']
            self.messages = data['messages']
            try:
                self.data = json.loads(data['data'])
            except:
                self.data = data['data']
        rfile.close()

    def write(self, socket):
        """ Package JSON to send over TCP to Unreal Server. """
        data = umsgpack.packb(self.to_json())
        length = len(data) + 8
        wfile = socket.makefile('wb', -1)
        wfile.write(struct.pack('!II', CONTROL_HEADER, length))
        wfile.write(data)
        wfile.close()


class StreamDataCommand(Message):
    """
    Request Unreal to open a Data Stream.

    destinationIp - IP of the client to accept the data stream. <unicode>
    destinationPort - Port of the client to accept the data stream. <int>
    protocol - TCP | UDP <unicode>
    action - Which action to send to the Server. (Found in monodrive.constants)
    format - What format to send the data. (Found in monodrive.constants)
    packet_size - The size of the receiving buffer
    """

    def __init__(self, device, id, destinationIp, destinationPort, protocol,
                 action, format=STREAM_DATA_FORMAT_RAW, packet_size=None, dropFrames=True):
        if packet_size is None:
            raise AttributeError('PacketSize is required.')  # TODO: make packet_size an argument

        super(StreamDataCommand, self).__init__(
            STREAM_DATA_COMMAND_UUID,
            {
                u"device_type": device,
                u"device_id": id,
                u"destination_address": destinationIp,
                u"destination_port": destinationPort,
                u"drop_frames": dropFrames,
                u"protocol": protocol,
                u"action": int(action),
                u"format": format,
                u"packet_size": packet_size
            })


class JSONConfigurationCommand(Message):
    def __init__(self, config_json, uuid):
        super(JSONConfigurationCommand, self).__init__(
            uuid,
            config_json)


class EgoControlCommand(Message):
    """ Send a command to control an Ego Vehicle. """

    def __init__(self, forward, right):
        super(EgoControlCommand, self).__init__(
            EGO_CONTROL_COMMAND_UID,
            {
                u"forward_amount": forward,
                u"right_amount": right
            })


class WaypointUpdateCommand(Message):
    """ Send a command to control an Ego Vehicle. """

    def __init__(self, position, lane):
        super(WaypointUpdateCommand, self).__init__(
            WAYPOINT_UPDATE_COMMAND_UID,
            {
                u"update_start_position": position,
                u"ego_vehicle_current_lane": lane
            })


class ScenarioModelCommand(Message):
    """ Send a command to run a scenario """

    def __init__(self, scenario):
        super(ScenarioModelCommand, self).__init__(
            SCENARIO_COMMAND_UID, scenario)


class ScenarioInitCommand(Message):
    """ Send a command to init a Scenario """

    def __init__(self, init):
        super(ScenarioInitCommand, self).__init__(
            SCENARIO_INIT_COMMAND_UID, init)
