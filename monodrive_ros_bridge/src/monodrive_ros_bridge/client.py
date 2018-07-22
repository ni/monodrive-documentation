#!/usr/bin/env python

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB), and the INTEL Visual Computing Lab.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.


"""
Ros Bridge node for mono simulator
"""

import rospy
import sys
import json
import yaml

from monodrive import SimulatorConfiguration, VehicleConfiguration


from monodrive_ros_bridge.bridge import MonoRosBridge
from monodrive_ros_bridge.bridge_with_rosbag import MonoRosBridgeWithBag


def main():

    # Simulator configuration defines network addresses for connecting to the simulator and material properties
    simulator_config = SimulatorConfiguration('simulator.json')
    sim_config_yaml = yaml.load(json.dumps(simulator_config.configuration))

    for param in sim_config_yaml:
        print param, ":", sim_config_yaml[param]
        rospy.set_param('/monodrive/' + param, sim_config_yaml[param])

    # Vehicle configuration defines ego vehicle configuration and the individual sensors configurations
    vehicle_config = VehicleConfiguration('demo.json')
    vehicle_config_yaml = yaml.load(json.dumps(vehicle_config.configuration))

    for param in vehicle_config_yaml:
        if param == 'sensors':
            for sensors in vehicle_config_yaml['sensors']:
                for p in sensors:
                    rospy.set_param('/monodrive/sensors/' + sensors['type'] + '/' + sensors['id'] + '/' + p, sensors[p])

        else:
            print "not included in params " + param

    import roslib
    roslib.load_manifest("rosparam")

    params = rospy.get_param('monodrive')
    host = params['host']
    port = params['port']
    num_episodes = params['Episodes']

    rospy.init_node("monodrive_client", anonymous=True)

    rospy.loginfo("Trying to connect to {host}:{port}".format(
        host=host, port=port))

    for episode in range(0, num_episodes):
        if rospy.is_shutdown():
            break
        rospy.loginfo("Starting Episode --> {}".format(episode))
        current_eps = '_episode' + '_' + str(episode)
        rospy.set_param(param_name='curr_episode',
                        param_value=current_eps)
        rospy.set_param(param_name='SimulatorConfig',
                        param_value='simulator.json')
        rospy.set_param(param_name='VehicleConfig',
                        param_value='single_camera.json')

        bridge_cls = MonoRosBridgeWithBag if rospy.get_param(
            'rosbag_fname', '') else MonoRosBridge
        with bridge_cls(params=params) as mono_ros_bridge:
            rospy.on_shutdown(mono_ros_bridge.on_shutdown)
            mono_ros_bridge.run()


if __name__ == "__main__":
    main()
