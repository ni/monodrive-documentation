from copy import deepcopy
import cv2
import numpy as np
import struct
import time

try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO


def timerfunc(func):
    """
    A timer decorator
    """

    def function_timer(*args, **kwargs):
        """
        A nested function for timing other functions
        """
        start = time.time()
        value = func(*args, **kwargs)
        end = time.time()
        runtime = end - start
        msg = "The runtime for {func} took {time} seconds to complete"
        print(msg.format(func=func.__name__,
                         time=runtime))
        return value

    return function_timer


class IntegerKeyBuffer(dict):
    def __setitem__(self, key, value):
        if not isinstance(key, int):
            raise ValueError('Integer key only')
        super(IntegerKeyBuffer, self).__setitem__(key, value)

    @property
    def sorted_keys(self):
        return sorted(self.keys())

    @property
    def image(self):
        _ = b''
        for i in self.sorted_keys:
            _ += self[i]
        return _


class CircularBuffer(object):
    def __init__(self, size, id, color=True):
        self.index = 0
        self.size = size
        self.data = []
        self.id = id
        self.image_mem = StringIO()
        self.color = color

    @property
    def image(self):
        buf = self.image_mem.getvalue()
        if not len(buf):
            raise BufferError
        if self.color:
            img_enc = cv2.IMREAD_COLOR
            return cv2.imdecode(np.asarray(bytearray(self.image_mem.getvalue()), dtype=np.uint8), img_enc)
        return np.asarray(bytearray(self.image_mem.getvalue()), dtype=np.uint8)

    def append(self, value):
        if len(self.data) == self.size:
            self.data[self.index] = value
        else:
            self.data.append(value)

        if self.color:
            img_rgb = cv2.cvtColor(value, cv2.COLOR_BGR2RGB)
            r, buf = cv2.imencode(".jpg", img_rgb)
            value = bytearray(buf)

        self.image_mem.seek(0)
        self.image_mem.write(value)
        self.index = (self.index + 1) % self.size

    def __getitem__(self, key):
        return self.data[key]

    def last(self):
        return self.data[-1]

    def is_empty(self):
        return len(self.data) == 0


class Frame(object):
    def __init__(self, _data):
        self.processed = False
        self.data = _data


class Packetizer(object):
    FRAME_START = 'imgstart'

    def __init__(self, delegate):
        self.delegate = delegate
        self.sequence_id = 0
        self.width = 0
        self.height = 0
        self.img_channels = 0
        self.first_header_size = 32
        self.normal_header_size = 6
        self.buffer = IntegerKeyBuffer()
        self.prev_frame = None
        self.expected_packet_count = -1
        self.frame_count = 0
        self.packet_count = 0
        self.packet_size = 1472  # TODO: Make this an argument so it is required
        self.frame_bytes_loaded = 0
        self.packet_payload = 0
        self.header_size = 0
        self.process_packet = True
        self.strict = True
        self.data = None
        self.image_threader = None
        self.frame_ready = False
        self.digest_frame_callback = None
        self.log_timestamp_callback = None
        self.should_patch_frame = False

        self.game_time = 0
        self.start_frame_bytes = bytes(Packetizer.FRAME_START)

    @property
    def calculated_image_size(self):
        return self.width * self.height * self.img_channels

    @property
    def calculated_header_length(self):
        return self.width * self.height * self.img_channels

    def get_expected_packet_count(self):
        if self.expected_packet_count == -1:
            self.expected_packet_count = self.calculate_expected_packet_count()

        return self.expected_packet_count

    def calculate_expected_packet_count(self):
        if self.calculated_image_size == 0:
            return -1

        # Work on turning this into a formula.
        packet_count = 1
        image_size = self.calculated_image_size - (self.packet_size - self.first_header_size)
        while image_size > 0:
            image_size = image_size - (self.packet_size - self.normal_header_size)
            packet_count = packet_count + 1

        return packet_count

    def process_header(self, data, first=False):
        if first:
            fmt_string = '!HIIHIfIHH'
            self.sequence_id, self.width, self.height, \
            self.img_channels, self.time_stamp, self.game_time, self.payload_size, \
            self.packet_payload, self.header_size = list(struct.unpack(fmt_string, data[8:36]))
            self.expected_packet_count = -1
            #print('header', self.sequence_id, self.width, self.height, self.calculated_image_size, self.payload_size)
            if self.calculated_image_size != self.payload_size:
                print("Received image Size does not match requested size: {0},{1},{2},{3},{4},{5}".format(
                    self.calculated_image_size, self.payload_size, self.width, self.height, self.img_channels, self.delegate.name))
        else:
            self.sequence_id, self.packet_payload, self.header_size = list(struct.unpack("!HHH", data[0:6]))

    def on_udp_received(self, data):
        start = data.startswith(bytes(Packetizer.FRAME_START))

        # this is just a check to catch error cases - we should have already digested the previous frame
        if start and self.packet_count != 0:
            self.frame_error()

        # process packet
        prev_game_time = self.game_time
        self.packet_count += 1
        self.process_header(data, start)
        self.buffer[self.sequence_id] = data[self.header_size:(self.packet_payload + self.header_size)]

        # additional processing for start frame
        if start:
            self.frame_count += 1
            # print("gametime prev:{0} cur:{1} diff:{2}".format(prev_game_time, self.game_time, game_time_dif))
            if self.delegate.log_timestamp_callback is not None:
                self.delegate.log_timestamp_delegate(self.time_stamp, self.game_time)

        # check if we completed our frame
        if self.packet_count == self.get_expected_packet_count():
            self.frame_complete()

    def on_udp_frame_error(self):
        self.frame_error()

    def frame_complete(self):
        # print("frame_complete: {0} len:{1}".format(self.delegate.name, len(self.buffer.image)))
        #print("packets: ", self.packet_count, "expected: ", self.expected_packet_count)
        self.prev_frame = deepcopy(self.buffer)
        self.send_digest_frame(self.buffer.image)
        self.packet_count = 0
        if not self.should_patch_frame:
            self.buffer = IntegerKeyBuffer()

    def frame_error(self):
        print("error: {0} did not receive complete frame. received:{1} expected:{2}".format(
            self.delegate.name, self.packet_count, self.get_expected_packet_count()))
        # todo - we need to check the clock mode - if we're blocking for client controlled stepping
        # todo - we need to signal the sensor that this frame did not complete

        # for now we will just send this patched frame or previous frame 
        # but we should skip frames if we know how to signal the sensor and can ask what the sensor would like to do
        if self.should_patch_frame and len(self.buffer) == self.get_expected_packet_count():
            self.send_digest_frame(self.buffer.image)
        else:
            # determine which packets in this frame were missed
            idx = 0
            while idx < self.get_expected_packet_count():
                if idx not in self.buffer:
                    # print("missing packet seq: ", idx)
                    # experimental - replace missing image frames with black pixels
                    if self.should_patch_frame:
                        self.buffer[idx] = bytearray(self.packet_size - self.header_size)
                idx += 1

            if self.should_patch_frame:
                # print("sending patched image: p:{0} l:{1}".format(len(self.buffer), len(self.buffer.image)))
                self.send_digest_frame(self.buffer.image)
            elif self.prev_frame is not None:
                self.send_digest_frame(self.prev_frame.image)
            else:
                print("error first frame had error - no previous frame to send")

        self.packet_count = 0
        if not self.should_patch_frame:
            self.buffer = IntegerKeyBuffer()

    def send_digest_frame(self, frame_image):
        # frame = np.array(bytearray(frame_image), dtype=np.uint8).reshape(self.height, self.width, self.img_channels)
        self.delegate.digest_frame_delegate(frame_image, self.time_stamp, self.game_time)
