from . import BaseModel, Story, Condition, Action


class Storyboard(BaseModel):
    def __init__(self, xml_data):
        actions = xml_data.find('Init').find('Actions')
        self.init_longitudinal_actions = Action.parse_longitudinal_actions(actions)
        self.init_position_actions = Action.parse_position_actions(actions)
        self.stories = BaseModel.parse_models(xml_data.findall('Story'), Story)
        # Seen this node as both End and EndCondition
        self.end_conditions = Condition.parse_conditions(xml_data.find('End'))

    @property
    def to_json(self):
        return {
            "init_longitudinal_actions": [act.to_json for act in self.init_longitudinal_actions],
            "init_position_actions": [act.to_json for act in self.init_position_actions],
            "stories": [s.to_json for s in self.stories],
            "end_conditions": [cond.to_json for cond in self.end_conditions]
        }

    @property
    def init_actions_json(self):
        init_actions = self.init_longitudinal_actions
        init_actions.extend(self.init_position_actions)
        return [act.to_json for act in init_actions]

