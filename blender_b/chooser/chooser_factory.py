from blender_b.chooser.random_all import RandomAll
from blender_b.chooser.random_in_blend_target import RandomInBlendTarget
from blender_b.chooser.choose_in_sti_range import ChooseInSTIRange
from opencog.logger import log
from util_b.general_util import BlLogger, get_class, \
    get_class_by_split_name

__author__ = 'DongMin Kim'

from opencog.type_constructors import *

from util_b import blending_util


class ChooserFactory(object):
    def __init__(self, a):
        self.a = a

        self.chooser_list = [
            RandomAll,
            RandomInBlendTarget,
            ChooseInSTIRange
        ]

        self.chooser_count = len(self.chooser_list)

    def get_chooser(self, id_or_name):
        if type(id_or_name) is str:
            for chooser in self.chooser_list:
                if str(chooser).find(id_or_name) != -1:
                    return chooser(self.a)
        else:
            return self.chooser_list[id_or_name](self.a)
