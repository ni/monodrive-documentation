#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB), and the INTEL Visual Computing Lab.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.



"""
Classes to handle mono sensors
"""
import math
import numpy as np
import tf

from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Imu, NavSatFix
from sensor_msgs.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header

from velodyne_msgs.msg import VelodynePacket

from monodrive.sensors.camera import Camera
from monodrive.sensors.lidar import Lidar
from monodrive.sensors import BaseSensor as Sensor

from monodrive_ros_bridge.transforms import mono_transform_to_ros_transform
from monodrive_ros_bridge.msg import BoundingBox, Rpm, Waypoint


cv_bridge = CvBridge(
)  # global cv bridge to convert image between opencv and monodrive_ros_bridge


class SensorHandler(object):
    """
    Generic Sensor Handler

    A sensor handler compute the associated monodrive_ros_bridge message to a mono sensor data, and the transform asssociated to the
    sensor.
    These messages are passed to a *process_msg_fun* which will take care of publishing them.
    """

    def __init__(self,
                 name,
                 sensor=None,
                 process_msg_fun=None):
        """
        :param name: sensor name
        :param sensor: monodrive sensor instance
        :param mono_settings: mono_settings object
        :param process_msg_fun: function to call on each new computed message
        """
        self.process_msg_fun = process_msg_fun
        self.sensor = sensor
        self.name = name

    def process_sensor_data(self, data, cur_time):
        """
        process a mono sensor data object

        Generate sensor message and transform message

        :param data: mono sensor data
        :param cur_time: current monodrive_ros_bridge simulation time
        :return:
        """
        self._compute_sensor_msg(data, cur_time)
        self._compute_transform(data, cur_time)

    def _compute_sensor_msg(self, data, cur_time):
        """
        Compute the monodrive_ros_bridge msg associated to mono data
        :param data: SensorData object
        :param cur_time: current monodrive_ros_bridge simulation time
        """
        raise NotImplemented

    def _compute_transform(self, data, cur_time):
        """
        Compute the tf msg associated to mono data

        :param data: SensorData object
        :param cur_time: current monodrive_ros_bridge simulation time
        """
        raise NotImplemented


class LidarHandler(SensorHandler):
    """
    Class to handle Lidar sensors
    """

    def __init__(self, name, sensor, **kwargs):
        super(LidarHandler, self).__init__(
            name, sensor=sensor, **kwargs)

    def _compute_sensor_msg(self, sensor_data, cur_time):
        for lidar_packet in sensor_data:
            new_sensor_data = bytes(lidar_packet)
            topic = 'velodyne_packets'
            self.process_msg_fun(topic, VelodynePacket(stamp=cur_time, data=new_sensor_data))

    def _compute_transform(self, sensor_data, cur_time):
        parent_frame_id = "base_link"
        child_frame_id = self.name

        t = TransformStamped()
        t.header.stamp = cur_time
        t.header.frame_id = parent_frame_id
        t.child_frame_id = child_frame_id
        t.transform = mono_transform_to_ros_transform(
            self.sensor.get_transform())

        # for some reasons lidar sends already rotated cloud,
        # so it is need to ignore pitch and roll
        r = t.transform.rotation
        quat = [r.x, r.y, r.z, r.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.process_msg_fun('tf', t)


class CameraHandler(SensorHandler):
    """
    Class to handle Camera sensors
    """

    def __init__(self, name, sensor, **kwargs):
        super(CameraHandler, self).__init__(
            name, sensor=sensor, **kwargs)

        self.topic_image = '/'.join([name, 'image_raw'])
        self.topic_cam_info = '/'.join([name, 'camera_info'])
        self.build_camera_info()

    def build_camera_info(self):
        """
        computing camera info

        camera info doesn't change over time
        """
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.name
        '''camera_info.width = self.mono_object.ImageSizeX
        camera_info.height = self.mono_object.ImageSizeY
        camera_info.distortion_model = 'plumb_bob'
        cx = self.mono_object.ImageSizeX / 2.0
        cy = self.mono_object.ImageSizeY / 2.0
        fx = self.mono_object.ImageSizeX / (
            2.0 * math.tan(self.mono_object.FOV * math.pi / 360.0))
        '''

        #TODO FIX THIS TO READ FROM THE VEHICLE_CONFIG
        camera_info.height = self.sensor.height
        camera_info.distortion_model = 'plumb_bob'
        cx = self.sensor.width
        cy = self.sensor.height
        fx = 512 / (2.0 * math.tan(45 * math.pi / 360.0))


        fy = fx
        camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        camera_info.D = [0, 0, 0, 0, 0]
        camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
        camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0]
        self._camera_info = camera_info

    def _compute_sensor_msg(self, sensor_data, cur_time):
        encoding = 'passthrough'

        data = np.array(bytearray(sensor_data['image']), dtype=np.uint8).reshape(self.sensor.height, self.sensor.width, 4)

        img_msg = cv_bridge.cv2_to_imgmsg(data, encoding=encoding)
        img_msg.header.frame_id = self.name
        img_msg.header.stamp = cur_time

        cam_info = self._camera_info
        cam_info.header = img_msg.header

        self.process_msg_fun(self.topic_cam_info, cam_info)
        self.process_msg_fun(self.topic_image, img_msg)

    def _compute_transform(self, sensor_data, cur_time):
        parent_frame_id = "base_link"
        child_frame_id = self.name

        t = TransformStamped()
        t.header.stamp = cur_time
        t.header.frame_id = parent_frame_id
        t.child_frame_id = child_frame_id

        # for camera we reorient it to look at the same axis as the opencv projection
        # in order to get easy depth cloud for RGBD camera
        t.transform = mono_transform_to_ros_transform(
            self.sensor.get_transform())

        rotation = t.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        quat_swap = tf.transformations.quaternion_from_matrix(
            [[0, 0, 1, 0],
            [-1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1]])
        quat = tf.transformations.quaternion_multiply(quat, quat_swap)

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.process_msg_fun('tf', t)


class ImuHandler(SensorHandler):
    def __init__(self, name, sensor, **kwargs):
        super(ImuHandler, self).__init__(
            name, sensor=sensor, **kwargs)
        self.seq = 0

    def _compute_sensor_msg(self, sensor_data, cur_time):
        self.seq += 1

        header = Header()
        header.stamp = cur_time
        header.frame_id = self.name
        header.seq = self.seq

        msg = Imu()
        msg.header = header
        msg.orientation.x = -1
        msg.orientation.y = -1
        msg.orientation.z = -1
        msg.orientation.w = -1
        msg.orientation_covariance = [-1, -1, -1, -1, -1, -1, -1, -1, -1]

        msg.angular_velocity.x = sensor_data['angular_velocity_vector'][0]
        msg.angular_velocity.y = sensor_data['angular_velocity_vector'][1]
        msg.angular_velocity.z = sensor_data['angular_velocity_vector'][2]
        msg.angular_velocity_covariance = [-1, -1, -1, -1, -1, -1, -1, -1, -1]

        msg.linear_acceleration.x = sensor_data['acceleration_vector'][0]
        msg.linear_acceleration.y = sensor_data['acceleration_vector'][1]
        msg.linear_acceleration.z = sensor_data['acceleration_vector'][2]
        msg.linear_acceleration_covariance = [-1, -1, -1, -1, -1, -1, -1, -1, -1]

        self.process_msg_fun('imu', msg)


    def _compute_transform(self, sensor_data, cur_time):
        parent_frame_id = "base_link"
        child_frame_id = self.name

        t = TransformStamped()
        t.header.stamp = cur_time
        t.header.frame_id = parent_frame_id
        t.child_frame_id = child_frame_id

        t.transform = mono_transform_to_ros_transform(
            self.sensor.get_transform())

        rotation = t.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        quat_swap = tf.transformations.quaternion_from_matrix(
            [[0, 0, 1, 0],
             [-1, 0, 0, 0],
             [0, -1, 0, 0],
             [0, 0, 0, 1]])
        quat = tf.transformations.quaternion_multiply(quat, quat_swap)

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.process_msg_fun('tf', t)


class GpsHandler(SensorHandler):
    def __init__(self, name, sensor, **kwargs):
        super(GpsHandler, self).__init__(
            name, sensor=sensor, **kwargs)

    def _compute_sensor_msg(self, sensor_data, cur_time):
        header = Header()
        header.stamp = cur_time
        header.frame_id = self.name

        msg = NavSatFix()
        msg.header = header

        msg.status.status = 0
        msg.status.service = 1
        msg.latitude = sensor_data['lat']
        msg.longitude = sensor_data['lng']
        msg.altitude = sensor_data['elevation']

        # todo, add covariance
        msg.position_covariance = [-1, -1, -1, -1, -1, -1, -1, -1, -1]
        msg.position_covariance_type = 0  # unknown

        self.process_msg_fun('gps', msg)


    def _compute_transform(self, sensor_data, cur_time):
        parent_frame_id = "base_link"
        child_frame_id = self.name

        t = TransformStamped()
        t.header.stamp = cur_time
        t.header.frame_id = parent_frame_id
        t.child_frame_id = child_frame_id

        t.transform = mono_transform_to_ros_transform(
            self.sensor.get_transform())

        rotation = t.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        quat_swap = tf.transformations.quaternion_from_matrix(
            [[0, 0, 1, 0],
             [-1, 0, 0, 0],
             [0, -1, 0, 0],
             [0, 0, 0, 1]])
        quat = tf.transformations.quaternion_multiply(quat, quat_swap)

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.process_msg_fun('tf', t)


class RpmHandler(SensorHandler):
    def __init__(self, name, sensor, **kwargs):
        super(RpmHandler, self).__init__(
            name, sensor=sensor, **kwargs)

    def _compute_sensor_msg(self, sensor_data, cur_time):
        header = Header()
        header.stamp = cur_time
        header.frame_id = self.name

        msg = Rpm()
        msg.header = header
        msg.wheel_number = sensor_data['wheel_number']
        msg.wheel_rpm = sensor_data['wheel_rpm']

        self.process_msg_fun('rpm', msg)


    def _compute_transform(self, sensor_data, cur_time):
        parent_frame_id = "base_link"
        child_frame_id = self.name

        t = TransformStamped()
        t.header.stamp = cur_time
        t.header.frame_id = parent_frame_id
        t.child_frame_id = child_frame_id

        t.transform = mono_transform_to_ros_transform(
            self.sensor.get_transform())

        rotation = t.transform.rotation
        quat = [rotation.x, rotation.y, rotation.z, rotation.w]
        quat_swap = tf.transformations.quaternion_from_matrix(
            [[0, 0, 1, 0],
             [-1, 0, 0, 0],
             [0, -1, 0, 0],
             [0, 0, 0, 1]])
        quat = tf.transformations.quaternion_multiply(quat, quat_swap)

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.process_msg_fun('tf', t)

