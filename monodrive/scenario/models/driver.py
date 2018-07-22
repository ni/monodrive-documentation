
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

from . import BaseModel, Utility


class Driver(BaseModel):
    def __init__(self, xml_data):
        self.name = xml_data.get('name')
        self.obey_traffic_lights = Utility.str_to_bool(xml_data.get('obeyTrafficLights'))
        self.obey_traffic_signs = Utility.str_to_bool(xml_data.get('obeyTrafficSigns'))
        self.steering_distance = float(xml_data.get('steeringDistance'))
        self.foresight_distance = float(xml_data.get('foresightDistance'))
        self.respond_to_tailgating = float(xml_data.get('respondToTailgating'))
        self.urge_to_overtake = float(xml_data.get('urgeToOvertake'))
        self.use_of_indicator = float(xml_data.get('useOfIndicator'))
        self.keep_right_rule = float(xml_data.get('keepRightRule'))
        self.lane_change_dynamic = float(xml_data.get('laneChangeDynamic'))
        self.speed_keeping = float(xml_data.get('speedKeeping'))
        self.lane_keeping = float(xml_data.get('laneKeeping'))
        self.distance_keeping = float(xml_data.get('distanceKeeping'))
        self.observe_speed_limits = float(xml_data.get('observeSpeedLimits'))
        self.curve_speed = float(xml_data.get('curveSpeed'))
        self.desired_deceleration = float(xml_data.get('desiredDeceleration'))
        self.desired_acceleration = float(xml_data.get('desiredAcceleration'))
        self.desired_velocity = float(xml_data.get('desiredVelocity'))
        self.politeness = float(xml_data.get('politeness'))
        self.alertness = float(xml_data.get('alertness'))
        self.adapt_to_vehicle_type = float(xml_data.get('adaptToVehicleType'))
        self.adapt_to_time_of_day = float(xml_data.get('adaptToTimeOfDay'))
        self.adapt_to_road_conditions = float(xml_data.get('adaptToRoadConditions'))
        self.adapt_to_weather_conditions = float(xml_data.get('adaptToWeatherConditions'))
        self.body = Body(xml_data.find('Body'))

    @property
    def to_json(self):
        return {
            "name": self.name,
            "obey_traffic_lights": self.obey_traffic_lights,
            "obey_traffic_signs": self.obey_traffic_signs,
            "steering_distance": self.steering_distance,
            "foresight_distance": self.foresight_distance,
            "respond_to_tailgating": self.respond_to_tailgating,
            "urge_to_overtake": self.urge_to_overtake,
            "use_of_indicator": self.use_of_indicator,
            "keep_right_rule": self.keep_right_rule,
            "lane_change_dynamic": self.lane_change_dynamic,
            "speed_keeping": self.speed_keeping,
            "lane_keeping": self.lane_keeping,
            "distance_keeping": self.distance_keeping,
            "observe_speed_limits": self.observe_speed_limits,
            "curve_speed": self.curve_speed,
            "desired_deceleration": self.desired_deceleration,
            "desired_acceleration": self.desired_acceleration,
            "desired_velocity": self.desired_velocity,
            "politeness": self.politeness,
            "alertness": self.alertness,
            "adapt_to_vehicle_type": self.adapt_to_vehicle_type,
            "adapt_to_time_of_day": self.adapt_to_time_of_day,
            "adapt_to_road_conditions": self.adapt_to_road_conditions,
            "adapt_to_weather_conditions": self.adapt_to_weather_conditions,
            "body": self.body.to_json
        }


class Body(BaseModel):
    def __init__(self, xml_data):
        self.weight = float(xml_data.get('weight'))
        self.height = float(xml_data.get('height'))
        self.eye_distance = float(xml_data.get('eyeDistance'))
        self.sex = xml_data.get('sex')

    @property
    def to_json(self):
        return {
            "weight": self.weight,
            "height": self.height,
            "eye_distance": self.eye_distance,
            "sex": self.sex
        }