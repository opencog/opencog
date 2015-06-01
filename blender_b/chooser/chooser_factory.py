from blender_b.chooser.random_in_blend_target import RandomInBlendTarget
from blender_b.debug_blender import DebugBlender
from blender_b.random_blender import RandomBlender
from opencog.logger import log
from util_b.general_util import BlendingLoggerForDebug

__author__ = 'DongMin Kim'

from opencog.type_constructors import *

from util_b import blending_util


class ChooserFactory(object):
    def __init__(self, atomspace):
        self.a = atomspace

        self.chooser_list = [
            RandomInBlendTarget(self.a)
        ]

        self.chooser_count = len(self.chooser_list)

    def get_blender(self, id_or_name):
        if type(id_or_name) is str:
            for blender in self.chooser_list:
                if str(blender) == id_or_name:
                    return blender
        else:
            return self.chooser_list[id_or_name]
