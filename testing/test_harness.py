#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import argparse
import inspect
import json

import testing.tests as testsuite


parser = argparse.ArgumentParser(description='monoDrive simulator automates test harness script')
parser.add_argument('--sim-config', default='simulator.json')
parser.add_argument('--vehicle-config', default='test.json')
parser.add_argument('--sim-path', help='path to simulator executable')
parser.add_argument('--exclude',
                    help='comma separated list of tests to exclude (tests not in list will be included)')
parser.add_argument('--include',
                    help='comma separated list of tests to include (tests not in list will be excluded)')


class TestHarness:
    def __init__(self, args):
        self.args = args


    def get_all_tests(self):
        tests = []
        base = testsuite.BaseTest.mro()[0]
        for name, cls in inspect.getmembers(testsuite):
            if name is not 'BaseTest' and inspect.isclass(cls) and base in cls.mro():
                tests.append(cls)
        return tests

    def launch_simulator(self):
        pass


if __name__ == "__main__":
    args = parser.parse_args()

    sim_config = SimulatorConfiguration(args.sim_config)
    vehicle_config = VehicleConfiguration(args.vehicle_config)
