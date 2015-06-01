from blender_b.chooser.random_all import RandomAll
from blender_b.chooser.random_in_blend_target import RandomInBlendTarget
from blender_b.chooser.random_in_sti_range import RandomInSTIRange
from opencog.logger import log
from util_b.general_util import BlendingLoggerForDebug

__author__ = 'DongMin Kim'

from opencog.type_constructors import *

from util_b import blending_util


class ChooserFactory(object):
    def __init__(self, atomspace):
        self.a = atomspace

        self.chooser_list = [
            RandomAll(self.a),
            RandomInBlendTarget(self.a),
            RandomInSTIRange(self.a)
        ]

        self.chooser_count = len(self.chooser_list)

    def get_chooser(self, id_or_name):
        if type(id_or_name) is str:
            for blender in self.chooser_list:
                if str(blender) == id_or_name:
                    return blender
        else:
            return self.chooser_list[id_or_name]
