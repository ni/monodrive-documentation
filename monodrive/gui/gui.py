from .widgets import *
from monodrive.sensors import *


Widgets = {
    BoundingBox: BoundingBoxWidget,
    Camera: CameraWidget,
    GPS: GPSWidget,
    IMU: IMUWidget,
    Lidar: LidarWidget,
    MultiCamera: MultiCameraWidget,
#    Radar: RadarWidget,
    RPM: RPMWidget,
    Waypoint: WaypointWidget
}

class Gui:
    def __init__(self, vehicle):
        self.widgets = []
        self.render_processes = []
        self.create_widgets(vehicle)

    def create_widget(self, sensor):
        if Widgets.get(sensor.__class__):
            return Widgets[sensor.__class__](sensor)
        return None

    def create_widgets(self, vehicle):
        sensors = vehicle.get_sensors()
        for sensor in sensors:
            if sensor.display_process:
                widget = self.create_widget(sensor)
                if widget is not None:
                    self.widgets.append(widget)

    def start(self):
        for widget in self.widgets:
            sensor = widget.sensor
            render_process_name = sensor.name + '_Render'
            render_process = Process(target=widget.rendering_main,
                                     name=render_process_name)
            render_process.daemon = True
            render_process.start()
            self.render_processes.append(render_process)

    def stop(self):
        [p.terminate() for p in self.render_processes]
