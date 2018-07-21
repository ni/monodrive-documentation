from copy import deepcopy


class BaseModel:
    @property
    def to_json(self):
        return deepcopy(vars(self))

    @staticmethod
    def parse_models(models_xml, model_class):
        models = []
        if models_xml:
            for model_xml in models_xml:
                model = model_class(model_xml)
                models.append(model)
        return models


from .utility import Vector3D, Vector2D, Rotation3D, CatalogReference, Utility
from .vehicle import Vehicle
from .driver import Driver
from .catalog import Catalog, CatalogType
from .actor import Actor
from .condition import Condition, ConditionByEntity, ConditionByState, ConditionByValue, ConditionType
from .position import Position, PositionByRoute, PositionByWorld, PositionType
from .action import Action, ActionLongitudinal, ActionLateral, ActionPosition, ActionRouting, ActionType
from .maneuver import Maneuver
from .event import Event
from .sequence import Sequence
from .act import Act
from .story import Story
from .storyboard import Storyboard
from .scenario import Scenario
from .utility import Utility
