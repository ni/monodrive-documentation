import xml.etree.ElementTree as ET

from monodrive.constants import *
from . import BaseModel, Driver, Vehicle


class CatalogType:
    DRIVER = 0
    VEHICLE = 1


class Catalog(BaseModel):
    def __init__(self, xml_data, entry_class, catalog_type):
        self.name = xml_data.tag
        self.type = catalog_type
        path = xml_data.find('Directory').get('path')
        path += ('.xosc')
        path_components = path.split('/')

        xml_file = open(os.path.join(BASE_PATH, 'scenarios', *path_components), 'r')
        if xml_file is not None:
            tree = ET.parse(xml_file)
            root = tree.getroot()
            entries = BaseModel.parse_models(root, entry_class)
            self.entries = dict(zip([e.name for e in entries], entries))

        else:
            print('Catalog Error File Path:' + path)

    @property
    def to_json(self):
        return {
            "name": self.name,
            "type": self.type,
            "entries": dict([(ent_name, ent.to_json) for ent_name, ent in self.entries.items()])
        }

    @staticmethod
    def parse_catalogs(catalogs_xml):
        catalogs = {}
        for catalog in catalogs_xml:
            cat = None
            if catalog.tag == "DriverCatalog":
                cat = Catalog(catalog, Driver, CatalogType.DRIVER)
            elif catalog.tag == "VehicleCatalog":
                cat = Catalog(catalog, Vehicle, CatalogType.VEHICLE)
            if cat is not None:
                catalogs[cat.name] = cat

        return catalogs

