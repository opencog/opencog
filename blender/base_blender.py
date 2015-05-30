from util.blending_util import BlendTargetCtlForDebug

__author__ = 'DongMin Kim'

from abc import ABCMeta, abstractmethod

from opencog.type_constructors import *

from util import blending_util


class BaseBlender(object):
    """
    :type a: opencog.atomspace_details.AtomSpace
    """
    __metaclass__ = ABCMeta

    def __init__(self, atomspace):
        self.a = atomspace
        self.a_blend_target = BlendTargetCtlForDebug().get_blend_target()
        self.last_status = 0

    def __str__(self):
        return 'BaseBlender'

    @abstractmethod
    def blend(self):
        pass

    @abstractmethod
    def get_last_status(self):
        pass
