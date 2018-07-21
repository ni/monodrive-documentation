# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB), and the INTEL Visual Computing Lab.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
RosBridge class with rosbag support
"""

import time
from datetime import datetime

from tf2_msgs.msg import TFMessage
import rosbag
import rospy
import os

from monodrive_ros_bridge.bridge import MonoRosBridge


class MonoRosBridgeWithBag(MonoRosBridge):
    def __init__(self, *args, **kwargs):
        super(MonoRosBridgeWithBag, self).__init__(*args, **kwargs)

        prefix, ext = os.path.splitext(rospy.get_param('rosbag_fname'))
        rosbag_fname = os.path.abspath(prefix + rospy.get_param('curr_episode'))
        self.bag = rosbag.Bag(rosbag_fname, mode='w')

    def send_msgs(self):
        for publisher, msg in self.msgs_to_publish:
            self.bag.write(publisher.name, msg, self.cur_time)

        tf_msg = TFMessage(self.tf_to_publish)
        self.bag.write('tf', tf_msg, self.cur_time)

        super(MonoRosBridgeWithBag, self).send_msgs()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        rospy.loginfo("Closing the bag file")
        self.bag.close()
        super(MonoRosBridgeWithBag, self).__exit__(exc_type, exc_value,
                                                    traceback)
