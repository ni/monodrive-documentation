
import time


class LabVehicleSensors(object):
    def __init__(self, simulator):
        self.ego_vehicle = None
        self.vehicle_config = None
        self.simulator = simulator
        self.restart_event = None

    def start_ego(self):
        from monodrive import VehicleConfiguration
        from monodrive.vehicles import TeleportVehicle
        # Setup Ego Vehicle
        vehicle_config = VehicleConfiguration('demo.json')
        self.ego_vehicle = TeleportVehicle(self.simulator, vehicle_config, self.restart_event, self.map_data)

        # Send Radar Waveform
        self.ego_vehicle.update_fmcw_in_config()
        self.ego_vehicle.start()

    def get_ego(self):
        return self.ego_vehicle


class LabVehicleControl(object):
    def __init__(self, ego_vehicle):
        self.ego_vehicle = ego_vehicle

    def step(self, control):
        self.ego_vehicle.step({'forward': control[0], 'right': control[1]})
        return control


class TestDrive(object):
    def __init__(self):
        self.map_data = None
        self.simulator = None
        self.restart_event = None
        self.gui = None
        self.ego_vehicle = None
        self.client = None
        pass

    def start_ego(self):
        self.ego_vehicle.start()

    def get_simulator(self):
        return self.simulator

    def step(self, control):
        self.ego_vehicle.step(self.client, {'forward': control[0], 'right': control[1]})
        return control

    def run_setup(self):
        from monodrive import SimulatorConfiguration, VehicleConfiguration, Simulator
        from monodrive.ui import GUI
        from monodrive.networking.client import Client



        # Simulator configuration defines network addresses for connecting to the simulator and material properties
        simulator_config = SimulatorConfiguration('simulator.json')
        self.client = Client((simulator_config.configuration["server_ip"],
                              simulator_config.configuration["server_port"]))

        if not self.client.isconnected():
            self.client.connect()
        # Vehicle configuration defines ego vehicle configuration and the individual sensors configurations
        vehicle_config = VehicleConfiguration('demo.json')

        self.simulator = Simulator(self.client, simulator_config)
        self.simulator.send_configuration()
        self.map_data = self.simulator.request_map()

        from monodrive import VehicleConfiguration
        from monodrive.vehicles import LV_Vehicle
        # Setup Ego Vehicle
        vehicle_config = VehicleConfiguration('demo.json')
        self.ego_vehicle = LV_Vehicle(simulator_config, vehicle_config, self.restart_event, self.map_data)

        # Send Radar Waveform
        self.ego_vehicle.update_fmcw_in_config()

        #helper = InterruptHelper()

        self.simulator.restart_event.clear()
        self.simulator.send_vehicle_configuration(vehicle_config)
        time.sleep(1)
        return True

    def start_sensor_streams(self):
        if not self.client.isconnected():
            self.client.connect()
        self.ego_vehicle.start_sensor_streaming(self.client)

    def stop_sensor_streams(self):
        if not self.client.isconnected():
            self.client.connect()
        self.ego_vehicle.stop_sensor_streaming(self.client)

    def close_connection(self):
        if self.client.isconnected():
            self.client.disconnect()

    def start_sensor_listening(self):
        self.ego_vehicle.start_sensor_listening()

    def start_gui(self):
        #self.gui = GUI(self.simulator)
        pass

    def stop_all(self):
        self.simulator.stop()


'''WRAPPERS FOR LABVIEW METHOD CALLS'''
s = TestDrive()


def set_up_simulator():
    try:
        s.run_setup()
    except Exception as e:
        raise ValueError(e)


def start_sensor_streams():
    try:
        s.start_sensor_streams()
    except Exception as e:
        raise ValueError(e)


def start_sensor_listening():
    try:
        s.start_sensor_listening()
    except Exception as e:
        raise ValueError(e)


def stop_sensor_streams():
    try:
        s.stop_sensor_streams()
    except Exception as e:
        raise ValueError(e)


def close_connection():
    try:
        s.close_connection()
    except Exception as e:
        raise ValueError(e)


def ego_step(control):
    control = control
    try:
        control = s.step(control)
    except Exception as e:
        raise ValueError(e)
    return control


def stop_all():
    stop_all()
    # Terminates vehicle and sensor processes

