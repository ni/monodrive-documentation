import xml.etree.ElementTree as ET
import os

from monodrive.constants import BASE_PATH

from . import BaseModel, Actor, Catalog, Storyboard


class Scenario(BaseModel):
    def __init__(self, xml_file_name):
        tree = ET.parse(open(os.path.join(BASE_PATH, 'scenarios', xml_file_name), 'r'))
        root = tree.getroot()
        self.catalogs = Catalog.parse_catalogs(root.find('Catalogs'))
        self.road_network = Scenario._parse_road_network(root.find('RoadNetwork'))
        self.actors = Actor.parse_actors(root.find('Entities'))
        self.storyboard = Storyboard(root.find('Storyboard'))

    @property
    def to_json(self):
        return {
            "catalogs": dict([(cat_name, cat.to_json) for cat_name, cat in self.catalogs.items()]),
            "road_network": self.road_network,
            "actors": [act.to_json for act in self.actors],
            "storyboard": self.storyboard.to_json
        }

    @property
    def init_scene_json(self):
        return {
            "catalogs": dict([(cat_name, cat.to_json) for cat_name, cat in self.catalogs.items()]),
            "road_network": self.road_network,
            "actors": [act.to_json for act in self.actors],
            "actions": self.storyboard.init_actions_json
        }

    @property
    def ego_vehicle_config(self):
        ego_actor = next((x for x in self.actors if x.name == 'Ego'), None)
        if ego_actor is None:
            raise Exception("No Ego Vehicle In Scenario")

        return self.get_entry_from_catalog_reference(ego_actor.config)

    @staticmethod
    def _parse_road_network(road_network_xml):
        scene_graph = road_network_xml.find('SceneGraph')
        scene_file_path = scene_graph.get('filepath')
        return scene_file_path

    def get_entry_from_catalog_reference(self, reference):
        if reference.catalog not in self.catalogs:
            raise Exception("Catalog Missing", reference.catalog)
        catalog = self.catalogs[reference.catalog]

        if reference.entry not in catalog.entries:
            raise Exception("Entry Missing from Catalog", reference.catalog, reference.entry)

        return catalog.entries[reference.entry]