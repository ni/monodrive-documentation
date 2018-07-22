
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

from . import BaseModel, Sequence, Condition


class Act(BaseModel):
    def __init__(self, xml_data):
        self.name = xml_data.get('name')
        self.sequence = Sequence(xml_data.find('Sequence'))
        conditions = xml_data.find('Conditions')
        start_cond = conditions.find('Start')
        self.start_conditions = Condition.parse_conditions(start_cond.find('ConditionGroup'))

    @property
    def to_json(self):
        return {
            "name": self.name,
            "sequence": self.sequence.to_json,
            "start_conditions": [c.to_json for c in self.start_conditions]
        }
