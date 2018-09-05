#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import argparse
import inspect
import json
import os
import subprocess
import sys
import threading
import time

from monodrive import configuration, simulator
import testing.tests as testsuite


parser = argparse.ArgumentParser(description='monoDrive simulator automates test harness script')
parser.add_argument('--sim-config', default='simulator.json')
parser.add_argument('--vehicle-config', default='test.json')
parser.add_argument('--exclude',
                    help='comma separated list of tests to exclude (tests not in list will be included)')
parser.add_argument('--include',
                    help='comma separated list of tests to include (tests not in list will be excluded)')
parser.add_argument('--no-launch', action="store_true", help='don\'t launch simulator')
parser.add_argument('simpath', metavar='simpath', help='path to simulator executable')


class AtomicBool:
    def __init__(self, value=False):
        self.lock = threading.Lock()
        self.value = value

    def get(self):
        self.lock.acquire()
        value = self.value
        self.lock.release()
        return value

    def set(self, value):
        self.lock.acquire()
        self.value = value
        self.lock.release()


class TestHarness:
    def __init__(self, args):
        self.args = args
        self.simulator_process_thread = None
        self.simulator_running = AtomicBool(False)
        self.simulator = simulator.Simulator(configuration.SimulatorConfiguration(self.args.sim_config))

    def get_all_tests(self):
        tests = []
        base = testsuite.BaseTest.mro()[0]
        for name, cls in inspect.getmembers(testsuite):
            if name is not 'BaseTest' and inspect.isclass(cls) and base in cls.mro():
                tests.append(cls(self))
        return tests

    def run_tests(self):
        tests = self.get_all_tests()
        if self.args.include:
            tests = filter(lambda t: t.id in self.args.include)
        elif self.args.exclude:
            tests = filter(lambda t: t.id not in self.args.exclude)

        tests = sorted(tests, key=lambda t: t.id)

        for test in tests:
            print("starting test %s" % test.id)
            test.run()
            print("result: %s" % test.result)
            print("========================================\n")

    def get_simulator_path(self):
        return self.args.simpath[0:self.args.simpath.rfind(os.sep)]

    def get_test_data_path(self):
        return os.path.join('.','tests','data')

    def get_simulator(self):
        return self.simulator

    def start_simulator(self):
        if self.args.no_launch:
            return

        if self.simulator_running.get():
            print('simulator is already running')
            return

        print('launching %s' % self.args.simpath)
        self.simulator_running.set(True)
        self.simulator_process_thread = threading.Thread(target=self.launch_simulator_)
        self.simulator_process_thread.start()

    def launch_simulator_(self):
        #print(subprocess.run(self.args.simpath, stdout=subprocess.PIPE, stderr=subprocess.PIPE))
        self.simulator_process = subprocess.Popen(self.args.simpath, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        while True:
            try:
                self.simulator_process.wait(timeout=.5)

                if self.simulator_process is None or self.simulator_process.returncode is not None:
                    break
            except subprocess.TimeoutExpired:
                if not self.simulator_running.get():
                    break

        self.simulator_running.set(False)
        print("simulator exited")

    def stop_simulator(self):
        if self.args.no_launch:
            return

        if self.simulator_running.get():
            if self.simulator_process is not None and self.simulator_process.returncode is None:
                print("shutting down simulator")
                if sys.platform == 'win32' or sys.platform == 'win64':
                    os.system("taskkill /f /t /pid %s" % self.simulator_process.pid)
                else:
                    self.simulator_process.kill()
                    self.simulator_process.wait()
            else:
                print("simulator isnt running")

            self.simulator_process = None
            self.simulator_process_thread = None
            self.simulator_running.set(False)

    def wait(self):
        if self.simulator_process_thread is not None:
            self.simulator_process_thread.join()


if __name__ == "__main__":
    args = parser.parse_args()
    env = TestHarness(args)
    env.run_tests()
    #sim_config = SimulatorConfiguration(args.sim_config)
    #vehicle_config = VehicleConfiguration(args.vehicle_config)
