from blender_b.debug_blender import DebugBlender
from blender_b.random_blender import RandomBlender
from util_b.general_util import BlConfig

__author__ = 'DongMin Kim'


class BlenderFinder(object):
    def __init__(self, a):
        self.a = a

        self.blender_list = {
            RandomBlender.__name__: RandomBlender,
            DebugBlender.__name__: DebugBlender
        }

        self.set_default_config()

    def __str__(self):
        return self.__class__.__name__

    def set_default_config(self):
        default_config = [
            ['BLENDER', 'RandomBlender']
        ]
        BlConfig().make_default_config(str(self), default_config)

    def get_blender(self, id_or_name=None):
        if id_or_name is None:
            id_or_name = BlConfig().get(str(self), 'BLENDER')

        blender = self.blender_list.get(str(id_or_name))
        if blender is not None:
            return blender(self.a)
        else:
            raise NameError('Blender not found.')

