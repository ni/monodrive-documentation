
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

from . import BaseModel, Condition, Action, ActionType


def parse_events(xml_data):
    events = []
    for event_xml in xml_data:
        event = Event(event_xml)
        events.append(event)
    return events


class Event(BaseModel):
    def __init__(self, xml_data):
        self.name = xml_data.get('name')
        self.priority = xml_data.get('priority')
        start_cond = xml_data.find('StartConditions')
        self.start_conditions = Condition.parse_conditions(start_cond.find('ConditionGroup'))
        self.action = Action.parse_event_action(xml_data.findall('Action'))
        self.is_finished = False
        self.is_active = False
        self.is_started = False
        self.start_simulation_time = None
        self.completion_simulation_time = None

    def start(self, vehicle_state):
        self.is_finished = False
        self.is_active = True
        self.start_simulation_time = vehicle_state.simulation_time

    def finish(self, vehicle_state):
        self.is_finished = True
        self.is_active = False
        self.completion_simulation_time = vehicle_state.simulation_time
        if self.action.type == ActionType.LATERAL_LANE_CHANGE:
            vehicle_state.target_lane_number = None

    def reset(self):
        self.is_finished = False
        self.is_active = False
        self.is_started = False
        self.start_simulation_time = None
        self.completion_simulation_time = None

    def is_event_complete(self, vehicle_state):
        if self.action.type == ActionType.LATERAL_LANE_CHANGE:
            if not self.is_started:
                # Minus because lane change right in scenario has target value of -1
                vehicle_state.target_lane_number = vehicle_state.estimated_lane_number - self.action.target_value
                self.is_started = True
                print('Starting Event {0} Changing from Lane {1} to {2}'.format(self.name, vehicle_state.estimated_lane_number, vehicle_state.target_lane_number))
            elif vehicle_state.estimated_lane_number == vehicle_state.target_lane_number:
                vehicle_state.event_complete()

    @property
    def to_json(self):
        return {
            "name": self.name,
            "priority": self.priority,
            "start_conditions": [c.to_json for c in self.start_conditions],
            "action": self.action.to_json
        }
