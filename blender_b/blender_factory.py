from blender_b.debug_blender import DebugBlender
from blender_b.random_blender import RandomBlender
from opencog.logger import log
from util_b.general_util import BlendingLoggerForDebug, get_class, \
    get_class_by_split_name

__author__ = 'DongMin Kim'

from opencog.type_constructors import *

from util_b import blending_util


class BlenderFactory(object):
    def __init__(self, a):
        self.a = a

        self.blender_list = [
            RandomBlender,
            DebugBlender
        ]

        self.blender_count = len(self.blender_list)

    def print_blender_list(self):
        BlendingLoggerForDebug().log('Please select blender number to use.')
        for i in range(self.blender_count):
            blender = self.blender_list[i]
            BlendingLoggerForDebug().log(str(i) + ': ' + str(blender))

    def ask_to_user(self):
        index = -1
        while (index < 0) or (index >= self.blender_count):
            index = input()

        return index

    def get_blender(self, id_or_name):
        if type(id_or_name) is str:
            for blender in self.blender_list:
                if str(blender).find(id_or_name) != -1:
                    return blender(self.a)
        else:
            return self.blender_list[id_or_name](self.a)
