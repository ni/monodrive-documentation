from . import BaseModel, Vector3D, Rotation3D


class PositionType:
    WORLD = 0
    ROUTE = 1


class Position(BaseModel):
    def __init__(self, xml_data, type):
        self.type = type

    @staticmethod
    def parse_position(xml_data):
        if xml_data.find('World') is not None:
            return PositionByWorld(xml_data)
        else:
            return PositionByRoute(xml_data)


class PositionByWorld(Position):
    def __init__(self, xml_data):
        super(PositionByWorld, self).__init__(xml_data, PositionType.WORLD)
        world = xml_data.find('World')
        self.location = Vector3D(world)
        self.orientation = Rotation3D(world)

    @property
    def to_json(self):
        d = super(PositionByWorld, self).to_json
        d["location"] = self.location.to_json
        d["orientation"] = self.orientation.to_json
        return d


class PositionByRoute(Position):
    def __init__(self, xml_data):
        super(PositionByRoute, self).__init__(xml_data, PositionType.ROUTE)
        lane_coord = xml_data.find('Route').find('Position').find('LaneCoord')
        self.path_s = float(lane_coord.get('pathS'))
        self.lane_id = int(lane_coord.get('laneId'))

