__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"


from . import BaseVehicle

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
    def __init__(self, client, simulator_config, vehicle_config, restart_event=None, road_map = None,**kwargs):
        super(TeleportVehicle, self).__init__(client, simulator_config, vehicle_config, restart_event)
        self.road_map = road_map
        self.throttle = 0.0
        self.steer = 0.0

        self.keyboard_thread = None
        self.keyboard_thread_running = False
        #self.start_keyboard_listener()

        self.name = "monoDrive tele control"
        self.instructions1 = "Select this window then"
        self.instructions2 = "use the ARROW keys drive the vehicle"

    def drive(self, sensors):
        #logging.getLogger("control").debug("Control Forward,Steer = {0},{1}".format(self.throttle,self.steer))
        if self.keyboard_thread is None:
            self.start_keyboard_listener()
        control = {'forward': self.throttle, 'right': self.steer}
        return control
    
    def start_keyboard_listener(self):
        self.keyboard_thread = threading.Thread(target=self._keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def stop(self):
        self.keyboard_thread_running = False
        super(TeleportVehicle, self).stop()
        self.keyboard_thread.join()

    def close(self):
        if self.keyboard_thread_running:
            self.keyboard_thread_running = False
            try:
                pygame.display.quit()
                pygame.quit()
            except: pass

    def _keyboard_listener(self):
        self.keyboard_thread_running = True

        pygame.init()
        pygame.mixer.quit()
        self.font1 = pygame.font.Font(None, 50)
        self.font2 = pygame.font.Font(None, 28)
        screen = pygame.display.set_mode((480, 360))

        self.draw(screen, False)

        while self.keyboard_thread_running:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if(self._get_keyboard_control(event.key)):
                        self.throttle = round(self.throttle, 2)
                        self.steer = round(self.steer, 2)
                        self.draw(screen)
                    else:
                        self.close()
                        self.restart_event.set()
                        break
            
            time.sleep(.1)

    def draw(self, screen, render_control=True):
        screen.fill((0, 0, 0))

        screen_rect = screen.get_rect()

        block = self.font2.render(self.name, True, (255, 255, 255))
        rect = block.get_rect()
        rect.left = (screen_rect.width - rect.width) / 2
        screen.blit(block, rect)

        block = self.font2.render(self.instructions1, True, (200, 200, 200))
        top = rect.bottom
        rect = block.get_rect()
        rect.top = top + 8
        rect.left = (screen_rect.width - rect.width) / 2
        screen.blit(block, rect)

        block = self.font2.render(self.instructions2, True, (200, 200, 200))
        top = rect.bottom
        rect = block.get_rect()
        rect.top = top + 2
        rect.left = (screen_rect.width - rect.width) / 2
        screen.blit(block, rect)

        if render_control:
            control = "throttle = {0} steering = {1}".format(self.throttle, self.steer)
            block = self.font1.render(control, True, (255, 255, 255))
            rect = block.get_rect()
            rect.center = screen_rect.center
            screen.blit(block, rect)

        pygame.display.flip()

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