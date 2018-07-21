from . import BaseModel, CatalogReference


class Actor(BaseModel):
    def __init__(self, xml_data):
        self.name = xml_data.get('name')
        config_ref_xml = xml_data.find('CatalogReference')
        self.config = CatalogReference(config_ref_xml)
        controller_ref_xml = xml_data.find('Controller').find('CatalogReference')
        self.controller = CatalogReference(controller_ref_xml)

    @property
    def to_json(self):
        return {
            "name": self.name,
            "config": self.config.to_json,
            "controller": self.controller.to_json
        }

    @staticmethod
    def parse_actor_names(actors_xml):
        actors = []
        for act_xml in actors_xml:
            name = act_xml.get('name')
            actors.append(name)

        return actors

    @staticmethod
    def parse_actors(actors_xml):
        actors = []
        for act_xml in actors_xml:
            actor = Actor(act_xml)
            actors.append(actor)

        return actors

