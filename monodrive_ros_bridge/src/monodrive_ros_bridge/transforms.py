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
Tool functions to convert transforms from mono to monodrive_ros_bridge coordinate system
"""
from geometry_msgs.msg import Transform, Pose
import tf


def mono_transform_to_ros_transform(mono_transform):
    """
    Convert a mono transform to a monodrive_ros_bridge transform
    :param mono_transform:
    :return: a monodrive_ros_bridge transform
    """
    transform_matrix = mono_transform.matrix

    x, y, z = tf.transformations.translation_from_matrix(transform_matrix)
    quat = tf.transformations.quaternion_from_matrix(transform_matrix)

    ros_transform = Transform()
    # remember that we go from left-handed system (unreal) to right-handed system (monodrive_ros_bridge)
    ros_transform.translation.x = x
    ros_transform.translation.y = -y
    ros_transform.translation.z = z

    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
    roll = -roll
    pitch = pitch
    yaw = -yaw

    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    ros_transform.rotation.x = quat[0]
    ros_transform.rotation.y = quat[1]
    ros_transform.rotation.z = quat[2]
    ros_transform.rotation.w = quat[3]

    return ros_transform


def mono_transform_to_ros_pose(mono_transform):
    """
    convert a mono transform to monodrive_ros_bridge pose msg
    :param mono_transform:
    :return: a monodrive_ros_bridge pose msg
    """
    transform_matrix = Transform(mono_transform).matrix

    x, y, z = tf.transformations.translation_from_matrix(transform_matrix)
    quat = tf.transformations.quaternion_from_matrix(transform_matrix)

    ros_transform = Transform()
    ros_transform.translation.x = x
    ros_transform.translation.y = y
    ros_transform.translation.z = z

    ros_transform.rotation.x = quat[0]
    ros_transform.rotation.y = quat[1]
    ros_transform.rotation.z = quat[2]
    ros_transform.rotation.w = quat[3]

    return ros_transform


def ros_transform_to_pose(ros_transform):
    """
    Util function to convert a monodrive_ros_bridge transform into a monodrive_ros_bridge pose

    :param ros_transform:
    :return: a monodrive_ros_bridge pose msg
    """
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = ros_transform.translation.x, \
                                                        ros_transform.translation.y, \
                                                        ros_transform.translation.z

    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = ros_transform.rotation.x, \
                                                                                     ros_transform.rotation.y, \
                                                                                     ros_transform.rotation.z, \
                                                                                     ros_transform.rotation.w
    return pose
