import os

from monodrive.constants import BASE_PATH
from monodrive.networking import messaging
from monodrive.scenario.models import Scenario


class ScenarioManager:

    def __init__(self, simulator):
        self.simulator = simulator

    def get_available_scenarios(self):
        """ Retrun a list of file objects that end with .json from the fixture directory. """
        return sorted([x for x in os.listdir(os.path.join(BASE_PATH, 'scenario')) if x.endswith(".xosc")])

    def load_scenario(self, file_name):
        xml_file = open(os.path.join(BASE_PATH, 'scenario', file_name), 'r')
        scen = Scenario(xml_file)
        return scen

    def send_scenario_init(self, scen):
        init = scen.init_scene_json
        msg = messaging.ScenarioInitCommand(init)
        return self.simulator.request(msg)

    def send_scenario(self, scen):
        json = scen.to_json
        msg = messaging.ScenarioModelCommand(scenario=json)
        return self.simulator.request(msg)

    def process_init_response(self, res):
        print(res)
        ready = False
        try:
            ready = res.data["ready"]
        except Exception as e:
            print("error parsing response\n")
            print(e)

        return ready



