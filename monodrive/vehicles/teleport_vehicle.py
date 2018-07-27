__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

import math
import numpy as np

from . import BaseVehicle
from monodrive.sensors import Waypoint, GPS

#for keyboard control
try:
    import pygame
    from pygame.locals import K_DOWN
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')


class TeleportVehicle(BaseVehicle):
    def __init__(self, simulator_config, vehicle_config, restart_event=None, **kwargs):
        super(TeleportVehicle, self).__init__(simulator_config, vehicle_config, restart_event)
        self.waypoint_sensor = Waypoint.get_sensor(self.sensors)
        self.gps_sensor = GPS.get_sensor(self.sensors)
        pygame.init()

    def drive(self, sensors, vehicle_state):
        
        forward = 0.0
        right = 0.0
        control = self._get_keyboard_control(pygame.key.get_pressed())

        return {
                'forward': control.throttle,
                'right': control.steer,
            }

    def _get_keyboard_control(self, keys):
        """
        Return a VehicleControl message based on the pressed keys. Return None
        if a new episode was requested.
        """
        if keys[K_r]:
            return None
        control = None
        if keys[K_LEFT] or keys[K_a]:
            control.steer = -1.0
        if keys[K_RIGHT] or keys[K_d]:
            control.steer = 1.0
        if keys[K_UP] or keys[K_w]:
            control.throttle = 1.0
        if keys[K_DOWN] or keys[K_s]:
            control.brake = 1.0
        if keys[K_SPACE]:
            control.hand_brake = True
        if keys[K_q]:
            self._is_on_reverse = not self._is_on_reverse
        if keys[K_p]:
            self._enable_autopilot = not self._enable_autopilot
        control.reverse = self._is_on_reverse
        return control