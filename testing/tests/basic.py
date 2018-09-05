import json
import os
import time

from . import BaseTest, TestResult
from monodrive.networking import messaging


class SimulatorConfigTest(BaseTest):
    def __init__(self, env):
        super(SimulatorConfigTest, self).__init__('basic.1', env)

    def before_test(self):
        pass

    def after_test(self):
        pass

    def test(self):
        self.env.start_simulator()
        time.sleep(10)
        #assert self.env.simulator_running.get()

        simulator = self.env.get_simulator()
        result = simulator.request(messaging.Message(messaging.SIMULATOR_STATUS_UUID))
        assert result is not None and result.data is not None

        self.env.stop_simulator()
        self.result = TestResult(True, result.to_json())
