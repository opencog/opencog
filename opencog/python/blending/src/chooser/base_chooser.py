from abc import ABCMeta, abstractmethod
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'

class BaseChooser(object):
    __metaclass__ = ABCMeta

    def __init__(self, atomspace):
        self.a = atomspace
        self.last_status = blending_status.UNKNOWN_ERROR
        self.default_config = {}
        self.make_default_config()

    def make_default_config(self):
        pass

    @abstractmethod
    def atom_choose(self, config):
        pass
