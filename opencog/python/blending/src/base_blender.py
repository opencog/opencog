from abc import ABCMeta, abstractmethod

__author__ = 'DongMin Kim'

class Blender(object):
    __metaclass__ = ABCMeta

    def __init__(self, atomspace):
        self.a = atomspace

    @abstractmethod
    def blend(self):
        pass
