__author__ = 'DongMin Kim'

from abc import ABCMeta, abstractmethod

from opencog.type_constructors import *

from util import blending_util


class BaseBlender(object):
    __metaclass__ = ABCMeta

    def __init__(self, atomspace):
        self.a = atomspace
        self.a_blend_target = \
            self.a.get_atoms_by_name(
                types.Atom,
                blending_util.BLEND_TARGET_NODE_NAME
            )[0]
        self.last_status = 0

    def __str__(self):
        return 'BaseBlender'

    @abstractmethod
    def blend(self):
        pass

    @abstractmethod
    def get_last_status(self):
        pass
