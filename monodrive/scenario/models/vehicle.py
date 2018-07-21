from monodrive.scenario.models.sensors import *


class Vehicle(BaseModel):
    def __init__(self, xml_data):
        self.id = xml_data.get('id')
        self.name = self.id
        self.anim_path = xml_data.get('anim_path')
        self.lane_number = int(xml_data.get('lane_number'))
        self.position = float(xml_data.get('position'))
        self.mesh_path = xml_data.get('mesh_path')
        self.clock_mode = int(xml_data.get('clock_mode'))
        self.bounding_data_on_radar_graph = Utility.str_to_bool(xml_data.get('bounding_data_on_radar_graph'))
        self.wheels = BaseModel.parse_models(xml_data.find('wheels'), Wheel)
        self.spawning_rotation = Rotation3D(xml_data.find('spawning_rotation'))
        self.multicamera_sensors = BaseModel.parse_models(xml_data.find('multicamera_sensors'), MultiCameraSensor)
        self.camera_sensors = BaseModel.parse_models(xml_data.find('camera_sensors'), CameraSensor)
        self.lidar_sensors = BaseModel.parse_models(xml_data.find('lidar_sensors'), LidarSensor)
        self.radar_sensors = BaseModel.parse_models(xml_data.find('radar_sensors'), RadarSensor)
        self.gps_sensors = BaseModel.parse_models(xml_data.find('gps_sensors'), GPSSensor)
        self.rpm_sensors = BaseModel.parse_models(xml_data.find('rpm_sensors'), RPMSensor)
        self.imu_sensors = BaseModel.parse_models(xml_data.find('imu_sensors'), IMUSensor)
        self.waypoint_sensors = BaseModel.parse_models(xml_data.find('waypoint_sensors'), WaypointSensor)
        self.bounding_box_sensors = BaseModel.parse_models(xml_data.find('bounding_box_sensors'), BoundingBoxSensor)

    @property
    def to_json(self):
        sensors = self.sensors
        return {
            "id": self.id,
            "anim_path": self.anim_path,
            "lane_number": self.lane_number,
            "clock_mode": self.clock_mode,
            "position": self.position,
            "mesh_path": self.mesh_path,
            "spawning_rotation": self.spawning_rotation.to_json,
            "wheels": [w.to_json for w in self.wheels],
            "sensors": [s.to_json for s in sensors]
        }

    @property
    def sensors(self):
        # Make more efficient / pythonic
        sensors = []
        sensors.extend(self.camera_sensors)
        sensors.extend(self.multicamera_sensors)
        sensors.extend(self.lidar_sensors)
        sensors.extend(self.radar_sensors)
        sensors.extend(self.gps_sensors)
        sensors.extend(self.rpm_sensors)
        sensors.extend(self.imu_sensors)
        sensors.extend(self.waypoint_sensors)
        sensors.extend(self.bounding_box_sensors)
        return sensors


class Wheel(BaseModel):
    def __init__(self, xml_data):
        self.id = xml_data.get('id')
        self.front = bool(xml_data.get('front'))
        self.wheel_number = int(xml_data.get('wheel_number'))
        self.offset = Vector3D(xml_data.find('offset'))

    @property
    def to_json(self):
        return {
            "id": self.id,
            "front": self.front,
            "wheel_number": self.wheel_number,
            "offset": self.offset.to_json
        }