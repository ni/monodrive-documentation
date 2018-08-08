__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"
import logging

import math
import numpy as np

from . import BaseVehicle
from monodrive.sensors import Waypoint, GPS


#for keyboard control
import threading
import time
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
    def __init__(self, simulator_config, vehicle_config, restart_event=None, road_map = None,**kwargs):
        super(TeleportVehicle, self).__init__(simulator_config, vehicle_config, restart_event)
        self.waypoint_sensor = Waypoint.get_sensor(self.sensors)
        self.gps_sensor = GPS.get_sensor(self.sensors)
        self.road_map = road_map
        self.throttle = 0.0
        self.steer = 0.0
        self.start_keyboard_listener()
        self.keyboard_thread = None
        self.keyboard_thread_running = True

    def drive(self, sensors, vehicle_state):
        logging.getLogger("control").debug("Control Forward,Steer = {0},{1}".format(self.throttle,self.steer))
        control = {'forward': self.throttle,'right': self.steer}
        return control
    
    def start_keyboard_listener(self):
        self.keyboard_thread = threading.Thread(target=self._keyboard_listener)
        self.keyboard_thread.start()

    def _keyboard_listener(self):
        pygame.init()
        screen = pygame.display.set_mode((480, 360))
        name = "monoDrive tele control"
        font = pygame.font.Font(None, 50)
        block = font.render(name, True, (255, 255, 255))
        while self.keyboard_thread_running:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if(self._get_keyboard_control(event.key)):
                        self.throttle = round(self.throttle, 2)
                        self.steer = round(self.steer, 2)
                        name = "throttle = {0} steering = {1}".format(self.throttle, self.steer)
                        screen.fill ((0, 0, 0))
                        block = font.render(name, True, (255, 255, 255))
                        rect = block.get_rect()
                        rect.center = screen.get_rect().center
                        screen.blit(block, rect)
                        pygame.display.flip()
                    else:
                        self.keyboard_thread_running = False
                        pygame.display.quit()
                        pygame.quit()
                        self.restart_event.set()
                        break

    def _get_keyboard_control(self, key):
        status = True
        if key == K_LEFT or key == K_a:
            self.steer += -.02
        if key == K_RIGHT or key == K_d:
            self.steer += .02
        if key == K_UP or key == K_w:
            self.throttle += .05
        if key == K_DOWN or key == K_s:
            self.throttle += -.05
        if key == K_r:
            self.simulator.restart_event.set()
        if key == K_q:
            status = False
        
        return status