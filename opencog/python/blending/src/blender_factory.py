from opencog.logger import log
from random_blender import RandomBlender

__author__ = 'DongMin Kim'


class BlenderFactory(object):
    def __init__(self, atomspace):
        self.a = atomspace

        self.blender_list = [
            RandomBlender(self.a)
        ]

    def get_blender(self, blender_name):
        for blender in self.blender_list:
            if str(blender) == blender_name:
                return blender
