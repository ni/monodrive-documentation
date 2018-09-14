#!/usr/bin/env python

__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

from threading import Event
import signal
import time


class InterruptHelper:
    def __init__(self):
        self.stop_event = Event()
        self.ignore_signals()

    #  child processes will inherit signal callbacks - this can be set to tell child processes to ignore kill
    def ignore_signals(self):
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)

    #  this will setup the callback for the interrupt handlers to be handled here
    def reset_signals(self):
        signal.signal(signal.SIGINT, self.on_shutdown)
        signal.signal(signal.SIGTERM, self.on_shutdown)

    def on_shutdown(self, signum, frame):
        #print("Kill signal received(%s)" % signum)
        self.stop_event.set()

    #  this will check both the app event and the kill signal and return true if either signal is set
    def wait(self, app_event, timeout=None):
        self.reset_signals()
        running = True
        signaled = False
        while running:
            if app_event.wait(.1 if timeout is None else timeout):
                running = False
                signaled = True
            else:
                if self.stop_event.is_set():
                    running = False
                    time.sleep(.1)  # some processes like ui might get killed anyway - this just pauses to let it happen
                    app_event.set()

            if running and timeout is not None:
                running = False

        return signaled
