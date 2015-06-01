from util_b.blending_util import BlendTargetCtlForDebug

__author__ = 'DongMin Kim'

from abc import ABCMeta, abstractmethod

from opencog.type_constructors import *

from util_b import blending_util


class BaseChooser(object):
    """
    :type a: opencog.atomspace_details.AtomSpace
    """
    __metaclass__ = ABCMeta

    def __init__(self, atomspace):
        self.a = atomspace
        self.a_blend_target = BlendTargetCtlForDebug().get_blend_target()
        self.last_status = 0

    def __str__(self):
        return 'BaseChooser'

    @abstractmethod
    def atom_choose(self, option):
        """
        :param option: dict
        :return: list
        """
        pass
