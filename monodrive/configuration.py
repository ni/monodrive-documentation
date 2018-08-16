import importlib
import inspect
import json
import os
from pkg_resources import resource_string


from monodrive.constants import BASE_PATH


class Configuration(object):
    def __init__(self, file_name):
        try:
            self.configuration = json.load(open(os.path.join(BASE_PATH, 'configurations', file_name)), 'r')
        except:
            file_path = os.path.join('..','configurations',file_name)
            self.configuration = json.loads(resource_string(__name__, file_path).decode('utf8'))

        self.id = self.configuration.get('id', None)
        self.name = file_name


class SimulatorConfiguration(Configuration):
    def __init__(self, file_name):
        super(SimulatorConfiguration, self).__init__(file_name)

        self.server_ip = self.configuration['server_ip']
        self.server_port = int(self.configuration['server_port'])
        self.client_ip = self.configuration['client_ip']
        self.client_settings = self.configuration['client_settings']
        del self.configuration['client_settings']

    @property
    def logger_settings(self):
        return self.client_settings['logger']

    @property
    def map_settings(self):
        return self.client_settings['map']
    
    @property
    def gui_settings(self):
        return self.client_settings['gui']


class VehicleConfiguration(Configuration):
    available_sensor_modules = {}

    def __init__(self, file_name):
        super(VehicleConfiguration, self).__init__(file_name)
        self.set_allowable_sensor_names()
        self.lane_number = self.configuration['lane_number']
        self.position = self.configuration['position']
        self.spawning_rotation = self.configuration['spawning_rotation']

        self.mesh_path = self.configuration['mesh_path']
        self.animation_path = self.configuration['anim_path']
        self.wheels = self.configuration['wheels']
        self.sensor_configuration = self.configuration['sensors']

        if not self.validate_sensors():
            raise AttributeError("Invalid Configuration. Validate the file is valid JSON.")

    @classmethod
    def init_from_json(cls, json):
        self = cls.__new__(cls)
        self.configuration = json
        self.set_allowable_sensor_names()
        self.lane_number = self.configuration['lane_number']
        self.position = self.configuration['position']
        self.spawning_rotation = self.configuration['spawning_rotation']

        self.mesh_path = self.configuration['mesh_path']
        self.animation_path = self.configuration['anim_path']
        self.wheels = self.configuration['wheels']
        self.sensor_configuration = self.configuration['sensors']

        if not self.validate_sensors():
            raise AttributeError("Invalid Configuration. Validate the file is valid JSON.")

        return self


    @property
    def available_sensor_classes(self):
        return self.available_sensor_modules.keys()

    def get_sensor_classes_from_module(self, name):
        return self.available_sensor_modules[name]['classes']

    def get_class(self, name):
        return self.available_sensor_modules[name]['cls']

    def set_allowable_sensor_names(self):
        sensor_module = importlib.import_module('monodrive.sensors')
        for name, obj in inspect.getmembers(sensor_module, inspect.isclass):
            if name.startswith('Base'):
                continue
            self.available_sensor_modules[obj.__name__] = {'classes': [], 'cls': obj}

    def validate_sensors(self):
        valid = True
        for sensor in self.sensor_configuration:
            if sensor['type'] not in self.available_sensor_classes:
                _m = 'Sensor Type: {0}'.format(sensor['type'])
                print('m', sensor, self.available_sensor_classes)
                _suggestions = [cls.__name__ for cls in self.available_sensor_classes
                                if sensor['type'] in cls.__name__]
                if len(_suggestions):
                    _m += ' Maybe you meant? {0}'.format(_suggestions)
                print(_m)
                valid = False
        return valid
