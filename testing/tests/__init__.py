class BaseTest(object):
    def __init__(self, id):
        self.id = id

    def run(self):
        raise NotImplementedError


from .basic import *
from .licensing import *