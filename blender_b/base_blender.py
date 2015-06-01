from blender_b.chooser.chooser_factory import ChooserFactory
from util_b.blending_util import BlendTargetCtlForDebug
from util_b.general_util import BlendingConfigLoader

__author__ = 'DongMin Kim'

from abc import ABCMeta, abstractmethod

from opencog.type_constructors import *

from util_b import blending_util


class BaseBlender(object):
    """
    :type a: opencog.atomspace_details.AtomSpace
    """
    __metaclass__ = ABCMeta

    def __init__(self, atomspace):
        self.a = atomspace
        if BlendingConfigLoader().is_use_blend_target:
            self.a_blend_target = BlendTargetCtlForDebug().get_blend_target()
        self.last_status = 0

        self.chooser_factory = ChooserFactory(self.a)

    def __str__(self):
        return 'BaseBlender'

    @abstractmethod
    def blend(self):
        pass

    @abstractmethod
    def get_last_status(self):
        pass
