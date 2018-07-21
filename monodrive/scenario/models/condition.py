from monodrive.scenario.models.utility import Utility
from . import BaseModel, Actor


class ConditionType:
    VALUE = 0
    STATE = 1
    ENTITY = 2


class Condition(BaseModel):
    def __init__(self, xml_data, type):
        self.type = int(type)
        self.name = xml_data.get('name')
        self.delay = float(xml_data.get('delay'))
        self.edge = xml_data.get('edge')

    def is_condition_ready(self, vehicle_state):
        raise NotImplementedError("To be implemented by subclass")

    def check_delay(self, event, vehicle_state):
        return (vehicle_state.simulation_time - event.completion_simulation_time) / 1000.0 >= self.delay

    @staticmethod
    def parse_condition(xml_data):
        if xml_data.find('ByValue'):
            return ConditionByValue(xml_data)
        elif xml_data.find('ByState'):
            return ConditionByState(xml_data)
        else:
            return ConditionByEntity(xml_data)

    @staticmethod
    def parse_conditions(xml_data):
        conditions = []
        for cond_xml in xml_data:
            cond = Condition.parse_condition(cond_xml)
            conditions.append(cond)
        return conditions

    @staticmethod
    def compare_rule_value(rule, condition_value, vehicle_state_value):
        if (rule == 'equal_to' and vehicle_state_value == condition_value) or \
                (rule == 'greater_than' and vehicle_state_value > condition_value) or \
                (rule == 'less_than' and vehicle_state_value < condition_value):
            return True
        return False


class ConditionByValue(Condition):
    def __init__(self, xml_data):
        super(ConditionByValue, self).__init__(xml_data, ConditionType.VALUE)
        sim_time = xml_data.find('ByValue').find("SimulationTime")
        self.value = int(sim_time.get('value'))
        self.rule = sim_time.get('rule')

    def is_condition_ready(self, vehicle_state):
        return Condition.compare_rule_value(self.rule, self.value, vehicle_state.simulation_elapsed_time)


class StateType:
    TERMINATION = 0
    START = 1
    COMMAND = 2
    SIGNAL = 3
    CONTROLLER = 4


class ConditionByState(Condition):
    def __init__(self, xml_data):
        super(ConditionByState, self).__init__(xml_data, ConditionType.STATE)
        state = xml_data.find('ByState')
        self.category = None
        self.state = None
        self.rule = None
        if state.find("AtStart") is not None:
            self.state_type = StateType.START
            self.comparing_name = state.find('AtStart').get('name')
            self.category = state.find('AtStart').get('type')
        elif state.find("AfterTermination") is not None:
            self.state_type = StateType.TERMINATION
            self.comparing_name = state.find('AfterTermination').get('name')
            self.category = state.find('AfterTermination').get('type')
            self.rule = state.find('AfterTermination').get('rule')
        elif state.find("Command") is not None:
            self.state_type = StateType.COMMAND
            self.comparing_name = state.find('Command').get('name')
        elif state.find("Signal") is not None:
            self.state_type = StateType.SIGNAL
            self.comparing_name = state.find('Signal').get('name')
            self.state = state.find('Signal').get('state')
        elif state.find("Controller") is not None:
            self.state_type = StateType.CONTROLLER
            self.comparing_name = state.find('Controller').get('name')
            self.state = state.find('Controller').get('state')

    def is_condition_ready(self, vehicle_state):
        # print('is_condition_ready()', self, self.name, vehicle_state.complete_events)
        if self.state_type == StateType.TERMINATION:
            # Condition passes when previous event completed has the same name as self.comparing_name
            for event in vehicle_state.complete_events:
                if event.action.name == self.comparing_name:
                    return self.check_delay(event, vehicle_state)
        return False


class ConditionByEntity(Condition):
    def __init__(self, xml_data):
        super(ConditionByEntity, self).__init__(xml_data, ConditionType.ENTITY)
        by_ent = xml_data.find('ByEntity')
        trig_ent = by_ent.find("TriggeringEntities")
        self.trigger_rule = trig_ent.get('rule')
        self.trigger_actors = Actor.parse_actor_names(trig_ent.find('Entity'))
        ent_cond = by_ent.find("EntityCondition")
        # There are 13 different conditions, only distance for now
        distance = ent_cond.find('Distance')
        self.value = float(distance.get('value'))
        self.free_space = Utility.str_to_bool(distance.get('freespace'))
        self.along_route = Utility.str_to_bool(distance.get('alongRoute'))
        self.rule = distance.get('rule')
        # Make a Position object, 8 different kinds of position
        pos = distance.find('Position').find('RelativeObject')
        self.object = pos.get('object')
        self.dx = float(pos.get('dx'))
        self.dy = float(pos.get('dy'))

    @property
    def to_json(self):
        d = super(ConditionByEntity, self).to_json
        d["trigger_rule"] = self.trigger_rule
        d["trigger_actors"] = [a for a in self.trigger_actors]
        d["value"] = self.value
        d["free_space"] = self.free_space
        d["along_route"] = self.along_route
        d["rule"] = self.rule
        d["object"] = self.object
        d["dx"] = self.dx
        d["dy"] = self.dy
        return d

    def is_condition_ready(self, vehicle_state):
        # print('is_condition_ready()', self, self.rule, self.value, vehicle_state.total_distance_traveled)
        return Condition.compare_rule_value(self.rule, self.value, vehicle_state.total_distance_traveled)
