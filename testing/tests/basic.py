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

        simulator = self.env.get_simulator()
        result = simulator.request(messaging.Message(messaging.SIMULATOR_STATUS_UUID))
        assert result is not None and result.data is not None
        assert result.data['configuration'] == "none"

        simulator.send_configuration()
        result = simulator.request(messaging.Message(messaging.SIMULATOR_STATUS_UUID))
        assert result is not None and result.data is not None
        assert result.data['configuration'] != "none"

        self.env.stop_simulator()
        self.result = TestResult(True, result.to_json())


class EgoConfigTest(BaseTest):
    def __init__(self, env):
        super(EgoConfigTest, self).__init__('basic.2', env)

    def before_test(self):
        pass

    def after_test(self):
        pass

    def test(self):
        self.env.start_simulator()
        time.sleep(10)

        simulator = self.env.get_simulator()
        simulator.send_configuration()
        simulator.request(messaging.Message(messaging.SIMULATOR_STATUS_UUID))
        simulator.send_vehicle_configuration(self.env.get_vehicle_config())

        time.sleep(10)
        result = simulator.request(messaging.Message(messaging.SIMULATOR_STATUS_UUID))
        assert result is not None and result.data is not None and result.data['vehicle_manager'] is not None
        vm = result.data['vehicle_manager']
        assert vm['ego']

        self.env.stop_simulator()
        self.result = TestResult(True, result.to_json())


class EgoControlTest(BaseTest):
    def __init__(self, env):
        super(EgoControlTest, self).__init__('basic.3', env)

    def before_test(self):
        pass

    def after_test(self):
        pass

    def test(self):
        self.env.start_simulator()
        time.sleep(10)

        simulator = self.env.get_simulator()
        simulator.send_configuration()
        simulator.request(messaging.Message(messaging.SIMULATOR_STATUS_UUID))
        simulator.send_vehicle_configuration(self.env.get_vehicle_config())

        time.sleep(10)
        result = simulator.request(messaging.Message(messaging.SIMULATOR_STATUS_UUID))
        vm = result.data['vehicle_manager']
        position = vm['ego']['location']
        speed = vm['ego']['speed']
        simulator.request(messaging.EgoControlCommand(2.5, 0.0))
        time.sleep(1)
        result = simulator.request(messaging.Message(messaging.SIMULATOR_STATUS_UUID))
        vm = result.data['vehicle_manager']

        assert vm['ego']['speed'] > speed
        assert vm['ego']['location']['x'] != position['x'] or vm['ego']['location']['y'] != position['y']

        self.env.stop_simulator()
        self.result = TestResult(True, result.to_json())