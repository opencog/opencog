from abc import ABCMeta, abstractmethod

__author__ = 'DongMin Kim'

class BaseChooser(object):
    __metaclass__ = ABCMeta

    def __init__(self, atomspace):
        self.a = atomspace
        self.last_status = 0

    def __str__(self):
        return 'BaseChooser'

    @abstractmethod
    def atom_choose(self, option):
        pass
