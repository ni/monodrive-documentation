from . import BaseModel


class Utility:
    @staticmethod
    def str_to_bool(s):
        if s == 'True' or s == 'true' or s == '1':
            return True
        else:
            return False


class Vector3D(BaseModel):
    def __init__(self, xml_data):
        self.x = float(xml_data.get('x')) if xml_data.get('x') is not None else 0.0
        self.y = float(xml_data.get('y')) if xml_data.get('y') is not None else 0.0
        self.z = float(xml_data.get('z')) if xml_data.get('z') is not None else 0.0


class Vector2D(BaseModel):
    def __init__(self, xml_data):
        self.x = float(xml_data.get('x')) if xml_data.get('x') is not None else 0.0
        self.y = float(xml_data.get('y')) if xml_data.get('y') is not None else 0.0


class Rotation3D(BaseModel):
    def __init__(self, xml_data):
        self.pitch = float(xml_data.get('pitch')) if xml_data.get('pitch') is not None else 0.0
        self.roll = float(xml_data.get('roll')) if xml_data.get('roll') is not None else 0.0
        self.yaw = float(xml_data.get('yaw')) if xml_data.get('yaw') is not None else 0.0


class CatalogReference(BaseModel):
    def __init__(self, xml_data):
        self.catalog = xml_data.get('catalogName')
        self.entry = xml_data.get('entryName')

