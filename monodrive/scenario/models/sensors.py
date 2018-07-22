
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

from . import BaseModel, Vector2D, Vector3D, Rotation3D, Utility


class SensorModelType:
    CAMERA = "Camera"
    MULTICAMERA = "MultiCamera"
    LIDAR = "Lidar"
    RADAR = "Radar"
    RPM = "RPM"
    GPS = "GPS"
    BOUNDING_BOX = "BoundingBox"
    WAYPOINT = "Waypoint"
    IMU = "IMU"


class BaseSensorModel(BaseModel):
    def __init__(self, xml_data, model_type):
        self.id = xml_data.get('id')
        self.type = model_type
        self.location = Vector3D(xml_data.find('location'))
        self.rotation = Rotation3D(xml_data.find('rotation'))
        self.fps = float(xml_data.get('fps'))
        self.listen_port = int(xml_data.get('listen_port'))
        self.packet_size = int(xml_data.get('packet_size'))
        self.sensor_process = Utility.str_to_bool(xml_data.get('sensor_process'))
        self.display_process = Utility.str_to_bool(xml_data.get('display_process'))

    @property
    def to_json(self):
        d = super(BaseSensorModel, self).to_json
        d["location"] = self.location.to_json
        d["rotation"] = self.rotation.to_json
        return d

    def __getitem__(self, x):
        return getattr(self, x)


class CameraSensor(BaseSensorModel):
    def __init__(self, xml_data, model_type=None):
        model_type = SensorModelType.CAMERA if model_type is None else model_type
        super(CameraSensor, self).__init__(xml_data, model_type)
        self.fps = float(xml_data.get('fps'))
        self.horizontal_fov_angle = float(xml_data.get('horizontal_fov_angle'))
        self.max_distance = float(xml_data.get('max_distance'))
        self.stream_dimensions = Vector2D(xml_data.find('stream_dimensions'))
        self.semantic_processing = Utility.str_to_bool(xml_data.get('semantic_processing'))
        self.hdmi_streaming = Utility.str_to_bool(xml_data.get('hdmi_streaming'))
    @property
    def to_json(self):
        d = super(CameraSensor, self).to_json
        d["horizontal_fov_angle"] = self.horizontal_fov_angle
        d["max_distance"] = self.max_distance
        d["stream_dimensions"] = self.stream_dimensions.to_json
        d["semantic_processing"] = self.semantic_processing
        d["hdmi_streaming"] = self.hdmi_streaming
        return d


class MultiCameraSensor(CameraSensor):
    def __init__(self, xml_data):
        super(MultiCameraSensor, self).__init__(xml_data, SensorModelType.MULTICAMERA)
        self.camera_ids = []
        ids = xml_data.findall('camera_ids')
        for id in ids:
            self.camera_ids.append(id.get('id'))


class IMUSensor(BaseSensorModel):
    def __init__(self, xml_data):
        super(IMUSensor, self).__init__(xml_data, SensorModelType.IMU)


class LidarSensor(BaseSensorModel):
    def __init__(self, xml_data):
        super(LidarSensor, self).__init__(xml_data, SensorModelType.LIDAR)
        self.horizontal_resolution = float(xml_data.get('horizontal_resolution'))
        self.max_distance = float(xml_data.get('max_distance'))
        self.n_lasers = int(xml_data.get('n_lasers'))
        self.reset_angle = float(xml_data.get('reset_angle'))
        self.vertical_fov_angle = float(xml_data.get('vertical_fov_angle'))


class RadarSensor(BaseSensorModel):
    def __init__(self, xml_data):
        super(RadarSensor, self).__init__(xml_data, SensorModelType.RADAR)
        self.fc = float(xml_data.get('fc'))
        self.fs = float(xml_data.get('fs'))
        self.max_velocity = float(xml_data.get('max_velocity'))
        self.num_samples_per_sweep = int(xml_data.get('num_samples_per_sweep'))
        self.num_sweeps = int(xml_data.get('num_sweeps'))
        self.range_max = float(xml_data.get('range_max'))
        self.range_resolution = float(xml_data.get('range_resolution'))
        self.sweep_num_for_range_max = float(xml_data.get('sweep_num_for_range_max'))
        self.receiver = RadarReceiver(xml_data.find('receiver'))
        self.transmitter = RadarTransmitter(xml_data.find('transmitter'))

    @property
    def to_json(self):
        d = super(RadarSensor, self).to_json
        d["fc"] = self.fc
        d["fs"] = self.fs
        d["max_velocity"] = self.max_velocity
        d["num_samples_per_sweep"] = self.num_samples_per_sweep
        d["num_sweeps"] = self.num_sweeps
        d["range_max"] = self.range_max
        d["range_resolution"] = self.range_resolution
        d["sweep_num_for_range_max"] = self.sweep_num_for_range_max
        d["receiver"] = self.receiver.to_json
        d["transmitter"] = self.transmitter.to_json
        return d


class RadarReceiver(BaseModel):
    def __init__(self, xml_data):
        self.aperture = float(xml_data.get('aperture'))
        self.gain = float(xml_data.get('gain'))
        self.kb = float(xml_data.get('kb'))
        self.nb = float(xml_data.get('nb'))
        self.nf = float(xml_data.get('nf'))
        self.noise_temp = float(xml_data.get('noise_temp'))


class RadarTransmitter(BaseModel):
    def __init__(self, xml_data):
        self.aperture = float(xml_data.get('aperture'))
        self.gain = float(xml_data.get('gain'))


class GPSSensor(BaseSensorModel):
    def __init__(self, xml_data):
        super(GPSSensor, self).__init__(xml_data, SensorModelType.GPS)


class RPMSensor(BaseSensorModel):
    def __init__(self, xml_data):
        super(RPMSensor, self).__init__(xml_data, SensorModelType.RPM)
        self.wheel_number = int(xml_data.get('wheel_number'))


class WaypointSensor(BaseSensorModel):
    def __init__(self, xml_data):
        super(WaypointSensor, self).__init__(xml_data, SensorModelType.WAYPOINT)
        self.point_delta = float(xml_data.get('point_delta'))
        self.total_points = int(xml_data.get('total_points'))


class BoundingBoxSensor(BaseSensorModel):
    def __init__(self, xml_data):
        super(BoundingBoxSensor, self).__init__(xml_data, SensorModelType.BOUNDING_BOX)
