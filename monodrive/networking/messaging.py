
from __future__ import absolute_import
from __future__ import print_function

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"



import json
import random
import struct
import sys

from monodrive.constants import *

from json import JSONEncoder
import codecs

class Message(object):
    """
    Base Message object that handles socket IO.

    A Client instance with an open connection to Unreal is required
    for a message object to successfully communicate.
    """

    def __init__(self, cls=u'', message=None):
        self.reference = random.randint(1,sys.maxsize)
        self.type = cls
        self.success = True
        self.message = message
        self.raw_data = None

    def to_json(self):
        return {
            u"reference": self.reference,
            u"type": self.type,
            u"success": self.success,
            u"message": self.message
        }

    @property
    def is_success(self):
        return self.success

    @property
    def is_sensor_ready(self):
        return self.message['sensor_ready'] if 'sensor_ready' in self.message else False

    @property
    def error_message(self):
        if not self.success:
            error = self.message
        else:
            error = None
        return error

    def __str__(self):
        return json.dumps(self.to_json())

    def read(self, socket):
        """ Read JSON from TCP data from the Unreal Server. """
        rbufsize = 0
        rfile = socket.makefile('rb', rbufsize)
        header = rfile.read(8)
        parsed_header = struct.unpack('!II', header)
        magic = parsed_header[0]
        length = parsed_header[1]

        #print("reading {0} bytes".format(length - 8))
        if magic == RESPONSE_HEADER and length > 8:
            self.raw_data = b''
            while len(self.raw_data) < length - 8:
                left = length - 8 - len(self.raw_data)
                self.raw_data += rfile.read(left)

            #print("received {0}".format(len(self.raw_data)))
            payload = json.loads(self.raw_data.decode("utf-8"))
            self.reference = payload.get("reference", 0)
            self.type = payload['type']
            self.success = payload['success']
            self.message = payload.get('message',{})

        rfile.close()

    def write(self, socket):
        """ Package JSON to send over TCP to Unreal Server. """
        #data = umsgpack.packb(self.to_json())
        payload = json.dumps(self.to_json()) #str(self.to_json()).encode('utf8')
        length = len(payload) + 8
        wfile = socket.makefile('wb', -1)
        wfile.write(struct.pack('!II', CONTROL_HEADER, length))
        wfile.write(payload.encode('utf8'))
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
    def __init__(self, uuid, config_json):
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


class MapCommand(Message):
    """ Send a command to request the Map. """

    def __init__(self, config):
        super(MapCommand, self).__init__(
            MAP_COMMAND_UID, config)


class SpawnCommand(Message):
    """ Send a command to spawn an object in the world. """

    def __init__(self, config):
        super(SpawnCommand, self).__init__(
            SPAWN_COMMAND_UID, config)


class MoveActorCommand(Message):
    """ Send a command to move an object in the world. """

    def __init__(self, config):
        super(MoveActorCommand, self).__init__(
            MOVE_ACTOR_COMMAND_UID, config)


class AttachSensorCommand(Message):
    """ Attach a sensor to the specified vehicle """

    def __init__(self, vehicle_id, sensor):
        super(AttachSensorCommand, self).__init__(
            ATTACH_SENSOR_COMMAND_UID,
            {
                "vehicle_id": vehicle_id,
                "sensor": sensor
            })


class DetachSensorCommand(Message):
    """ Attach a sensor to the specified vehicle """

    def __init__(self, vehicle_id, sensor):
        super(DetachSensorCommand, self).__init__(
            DETACH_SENSOR_COMMAND_UID,
            {
                "vehicle_id": vehicle_id,
                "sensor": sensor
            })

