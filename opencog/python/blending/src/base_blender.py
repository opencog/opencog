from abc import ABCMeta, abstractmethod

__author__ = 'DongMin Kim'

class BaseBlender(object):
    __metaclass__ = ABCMeta

    def __init__(self, atomspace):
        self.a = atomspace
        self.last_status = 0

    def __str__(self):
        return 'BaseBlender'

    @abstractmethod
    def blend(self):
        pass

    @abstractmethod
    def get_last_status(self):
        pass
