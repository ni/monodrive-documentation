from . import BaseModel, Actor, Maneuver


class Sequence(BaseModel):
    def __init__(self, xml_data):
        self.name = xml_data.get('name')
        self.executions = int(xml_data.get('numberOfExecutions'))
        self.actors = Actor.parse_actor_names(xml_data.find('Actors'))
        m = xml_data.find('Maneuver')
        # Some Sequences do not have a Maneuver
        if m:
            self.maneuver = Maneuver(xml_data.find('Maneuver'))
        else:
            self.maneuver = None

    @property
    def to_json(self):
        if self.maneuver:
            return {
                "name": self.name,
                "executions": self.executions,
                "actors": [a for a in self.actors],
                "maneuver": self.maneuver.to_json
            }
        else:
            return {
                "name": self.name,
                "executions": self.executions,
                "actors": [a for a in self.actors]
            }

