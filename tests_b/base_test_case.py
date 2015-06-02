from util_b.blending_util import BlendTargetCtlForDebug
from util_b.general_util import BlConfig

__author__ = 'DongMin Kim'

from abc import ABCMeta, abstractmethod

from opencog.type_constructors import *

from util_b import blending_util


class BaseTestCase(object):
    """
    :type a: opencog.atomspace_details.AtomSpace
    """
    __metaclass__ = ABCMeta

    def __init__(self, a):
        self.a = a
        self.atom_list_for_debug = []
        self.link_list_for_debug = []
        self.default_atom_tv = TruthValue(0.9, 0.8)
        self.default_link_tv = TruthValue(0.7, 0.6)

        if BlConfig().is_use_blend_target:
            self.a_blend_target = BlendTargetCtlForDebug().get_blend_target()

    def __str__(self):
        return 'BaseTestCase'

    @abstractmethod
    def make(self):
        pass
