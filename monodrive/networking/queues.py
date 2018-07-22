
__author__ = "monoDrive"
__copyright__ = "Copyright (C) 2018 monoDrive"
__license__ = "MIT"
__version__ = "1.0"

try:
    from queue import Empty, Full
except:
    from Queue import Empty, Full

import time
import multiprocessing as mp
from multiprocessing.queues import Queue as sq
import os
import sys


# We could probably implement a LifoQueue but it would be a little more difficult with the multiprocessing Queue

#
# multiprocessing Queue implementation that enforces the maxsize of 1 by replacing existing data
#


class SingleQueue(sq):

    def __init__(self):
        if sys.platform == "Windows":
            ctx = mp.get_context()
            super(SingleQueue, self).__init__(maxsize=1, ctx=ctx)
        else:
            super(SingleQueue, self).__init__(maxsize=1)         
        self._qlock = mp.RLock()

    def __getstate__(self):
        return self._qlock, super(SingleQueue, self).__getstate__()

    def __setstate__(self, state):
        self._qlock, state = state
        super(SingleQueue, self).__setstate__(state)

    def put(self, obj, block=True, timeout=None):
        if not self._qlock.acquire(True, None):
            raise Full
        try:
            self.__internal_put(obj, block, timeout)
        finally:
            self._qlock.release()

    def __internal_put(self, obj, block=True, timeout=None):
        if self.full():
            self.get(False)
        super(SingleQueue, self).put(obj, block, timeout)

    def get(self, block=True, timeout=None):
        res = None
        needs_loop = True
        while needs_loop:
            self.__wait_for_data(block)

            if not self._qlock.acquire(True, None):
                raise Empty

            try:
                res = self.__internal_get(block, timeout)
            finally:
                self._qlock.release()
            needs_loop = block and res is None

        return res

    def __internal_get(self, block, timeout):
        res = None
        if not self.empty():
            res = super(SingleQueue, self).get(block, timeout)
        return res

    def __wait_for_data(self, block):
        if block:
            while self.empty():
                time.sleep(0.01)

    def peek(self, block=True, timeout=None):
        res = None
        needs_loop = True
        while needs_loop:
            self.__wait_for_data(block)

            if not self._qlock.acquire(True, None):
                raise Empty

            try:
                res = self.__internal_get(block, timeout)
                if res is not None:
                    self.__internal_put(res, block, timeout)
            finally:
                self._qlock.release()
            needs_loop = block and res is None

        return res
