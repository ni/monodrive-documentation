
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

from . import BaseModel, CatalogReference, Position


class ActionType:
    LONGITUDINAL = 0
    POSITION = 1
    ROUTING = 2
    LATERAL_LANE_CHANGE = 3
    LATERAL_LANE_OFFSET = 4
    LATERAL_DISTANCE = 5


class Action(BaseModel):
    def __init__(self, xml_data, type):
        self.type = type
        self.name = xml_data.get('name')
        self.is_active = False
        self.is_finished = False

    @staticmethod
    def parse_event_action(xml_data):
        xml_data = xml_data[0]
        if xml_data.find('Private').find('Lateral').find('LaneChange'):
            return ActionLateralLaneChange(xml_data)
        elif xml_data.find('Private').find('Lateral').find('LaneOffset'):
            return ActionLateralLaneOffset(xml_data)
        elif xml_data.find('Private').find('Lateral').find('Distance'):
            return ActionLateralDistance(xml_data)
        elif xml_data.find('Private').find('Routing'):
            return ActionRouting(xml_data)
        return None

    @staticmethod
    def parse_longitudinal_actions(xml_data):
        actions = []
        for init_action in xml_data:
            actor_name = init_action.get('object')
            actions_xml = init_action.findall('Action')
            for action_xml in actions_xml:
                if action_xml.find('Longitudinal'):
                    action = ActionLongitudinal(action_xml, actor_name)
                    actions.append(action)

        return actions

    @staticmethod
    def parse_position_actions(xml_data):
        actions = []
        for init_action in xml_data:
            actor_name = init_action.get('object')
            actions_xml = init_action.findall('Action')
            for action_xml in actions_xml:
                if action_xml.find('Position'):
                    action = ActionPosition(action_xml, actor_name)
                    actions.append(action)

        return actions


class ActionLateral(Action):
    def __init__(self, xml_data, lateral_type):
        super(ActionLateral, self).__init__(xml_data, lateral_type)

        action_data = xml_data.find('Private').find('Lateral').find(self.node_name)
        self.parse_dynamics(action_data.find('Dynamics'))

        self.target_object = action_data.find('Target').find('Relative').get('object')
        self.target_value = int(action_data.find('Target').find('Relative').get('value'))

    def parse_dynamics(self, dynamics):
        raise NotImplementedError("To be implemented")

    @property
    def node_name(self):
        raise NotImplementedError("To be implemented")


class ActionLateralLaneChange(ActionLateral):
    def __init__(self, xml_data):
        super(ActionLateralLaneChange, self).__init__(xml_data, ActionType.LATERAL_LANE_CHANGE)
        self.time = None
        self.distance = None
        self.shape = None

    def parse_dynamics(self, dynamics_data):
        self.time = dynamics_data.get('time')
        self.distance = dynamics_data.get('distance')
        self.shape = dynamics_data.get('shape')

    @property
    def node_name(self):
        return 'LaneChange'


class ActionLateralLaneOffset(ActionLateral):
    def __init__(self, xml_data):
        super(ActionLateralLaneOffset, self).__init__(xml_data, ActionType.LATERAL_LANE_OFFSET)
        self.maxLateralAcc = None
        self.duration = None
        self.shape = None

    def parse_dynamics(self, dynamics_data):
        self.maxLateralAcc = dynamics_data.get('maxLateralAcc')
        self.duration = dynamics_data.get('duration')
        self.shape = dynamics_data.get('shape')

    @property
    def node_name(self):
        return 'LaneOffset'


class ActionLateralDistance(ActionLateral):
    def __init__(self, xml_data):
        super(ActionLateralDistance, self).__init__(xml_data, ActionType.LATERAL_DISTANCE)
        self.maxAcceleration = None
        self.maxDeceleration = None
        self.maxSpeed = None

    def parse_dynamics(self, dynamics_data):
        # Needs to be tested, there's a 'Limited' node before this I believe, no examples
        self.maxAcceleration = dynamics_data.get('maxAcceleration')
        self.maxDeceleration = dynamics_data.get('maxDeceleration')
        self.maxSpeed = dynamics_data.get('maxSpeed')

    @property
    def node_name(self):
        return 'Distance'


class ActionLongitudinal(Action):
    def __init__(self, xml_data, actor_name):
        super(ActionLongitudinal, self).__init__(xml_data, ActionType.LONGITUDINAL)
        self.name = "init"
        self.actor_name = actor_name
        speed = xml_data.find('Longitudinal').find('Speed')
        dynamics = speed.find('Dynamics')
        self.shape = dynamics.get('shape')
        self.rate = float(dynamics.get('rate')) if dynamics.get('rate') is not None else 0
        self.target_value = float(speed.find('Target').find('Absolute').get('value'))


class ActionPosition(Action):
    def __init__(self, xml_data, actor_name):
        super(ActionPosition, self).__init__(xml_data, ActionType.POSITION)
        self.name = "Position"
        self.actor_name = actor_name
        self.position = Position.parse_position(xml_data.find('Position'))

    @property
    def to_json(self):
        d = super(ActionPosition, self).to_json
        d["actor_name"] = self.actor_name
        d["position"] = self.position.to_json
        return d


class ActionRouting(Action):
    def __init__(self, xml_data):
        super(ActionRouting, self).__init__(xml_data, ActionType.ROUTING)
        trajectory = xml_data.find('Private').find('Routing').find('FollowTrajectory')
        config_ref_xml = trajectory.find('CatalogReference')
        self.trajectory_reference = CatalogReference(config_ref_xml)
        self.lateral_purpose = trajectory.find('Lateral').get('purpose')

    @property
    def to_json(self):
        d = super(ActionRouting, self).to_json
        d["trajectory_reference"] = self.trajectory_reference.to_json
        d["lateral_purpose"] = self.lateral_purpose
        return d
