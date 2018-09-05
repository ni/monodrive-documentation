import json
import os
import time

from . import BaseTest, TestResult
from monodrive.networking import messaging


class GoodLicenseTest(BaseTest):
    def __init__(self, env):
        super(GoodLicenseTest, self).__init__('licensing.1', env)

    def before_test(self):
        license = open(os.path.join(self.env.get_test_data_path(), 'license.txt')).read()
        print(json.loads(license)['subscription_id'])

        license_path = os.path.join(self.env.get_simulator_path(), 'license.txt')
        if os._exists(license_path):
            os.rename(license_path, license_path+'.tmp') # rename existing license file

        f = open(os.path.join(self.env.get_simulator_path(), 'license.txt'), 'w') # save test license file
        f.write(license)
        f.close()

    def after_test(self):
        license_path = os.path.join(self.env.get_simulator_path(), 'license.txt')
        os.remove(license_path) # remove test license.txt
        if os._exists(license_path + '.tmp'):
            os.rename(license_path + '.tmp', license_path) # rename old license file

    def test(self):
        self.env.start_simulator()
        time.sleep(10)
        #assert self.env.simulator_running.get()

        simulator = self.env.get_simulator()
        result = simulator.request(messaging.Message(messaging.SIMULATOR_STATUS_UUID))
        assert result is not None and result.data is not None
        assert result.data['license']['valid']

        self.env.stop_simulator()
        self.result = TestResult(True, result.to_json())


class MissingLicenseTest(BaseTest):
    def __init__(self, env):
        super(MissingLicenseTest, self).__init__('licensing.2', env)

    def before_test(self):
        license_path = os.path.join(self.env.get_simulator_path(), 'license.txt')
        if os._exists(license_path):
            os.rename(license_path, license_path + '.tmp')  # rename old license file

    def after_test(self):
        license_path = os.path.join(self.env.get_simulator_path(), 'license.txt')
        if os._exists(license_path + '.tmp'):
            os.rename(license_path + '.tmp', license_path)  # rename old license file

    def test(self):
        self.env.start_simulator()
        time.sleep(10)

        simulator = self.env.get_simulator()
        result = simulator.request(messaging.Message(messaging.SIMULATOR_STATUS_UUID))
        assert result is not None and result.data is not None
        assert result.data['license']['valid'] is False

        self.env.stop_simulator()
        self.result = TestResult(True, result.to_json())


class BadLicenseTest(BaseTest):
    def __init__(self, env):
        super(BadLicenseTest, self).__init__('licensing.3', env)

    def before_test(self):
        license = open(os.path.join(self.env.get_test_data_path(), 'license.txt')).read()
        print(json.loads(license)['subscription_id'])

        license_path = os.path.join(self.env.get_simulator_path(), 'license.txt')
        if os._exists(license_path):
            os.rename(license_path, license_path+'.tmp') # rename existing license file

        f = open(os.path.join(self.env.get_simulator_path(), 'license.txt'), 'w') # save test license file
        lic = json.loads(license)
        lic['data'] = lic['data'][1:] # remove char from license data
        f.write(json.dumps(lic))
        f.close()

    def after_test(self):
        license_path = os.path.join(self.env.get_simulator_path(), 'license.txt')
        os.remove(license_path)  # remove test license.txt
        if os._exists(license_path + '.tmp'):
            os.rename(license_path + '.tmp', license_path)  # rename old license file

    def test(self):
        self.env.start_simulator()
        time.sleep(10)

        simulator = self.env.get_simulator()
        result = simulator.request(messaging.Message(messaging.SIMULATOR_STATUS_UUID))
        assert result is not None and result.data is not None
        assert result.data['license']['valid'] is False

        self.env.stop_simulator()
        self.result = TestResult(True, result.to_json())