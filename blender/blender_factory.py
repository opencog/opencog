from blender.debug_blender import DebugBlender
from blender.random_blender import RandomBlender
from opencog.logger import log

__author__ = 'DongMin Kim'

from opencog.type_constructors import *

from util import blending_util


class BlenderFactory(object):
    def __init__(self, atomspace):
        self.a = atomspace

        self.blender_list = [
            RandomBlender(self.a),
            DebugBlender(self.a)
        ]

        self.blender_count = len(self.blender_list)

    def print_blender_list(self):
        log.warn('Please select blender number to use.')
        for i in range(self.blender_count):
            blender = self.blender_list[i]
            log.warn(str(i) + ': ' + str(blender))

    def ask_to_user(self):
        index = -1
        while (index < 0) or (index >= self.blender_count):
            index = input()

        return index

    def get_blender(self, id_or_name):
        if type(id_or_name) is str:
            for blender in self.blender_list:
                if str(blender) == id_or_name:
                    return blender
        else:
            return self.blender_list[id_or_name]
