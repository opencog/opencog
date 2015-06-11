from blender_b.decider.decide_best_sti import DecideBestSTI
from blender_b.decider.decide_null import DecideNull
from blender_b.decider.decide_random import DecideRandom
from util_b.general_util import BlConfig

__author__ = 'DongMin Kim'


class DeciderFinder(object):
    def __init__(self, a):
        self.a = a

        self.decider_list = {
            DecideNull.__name__: DecideNull,
            DecideRandom.__name__: DecideRandom,
            DecideBestSTI.__name__: DecideBestSTI
        }

        self.set_default_config()

    def __str__(self):
        return self.__class__.__name__

    def set_default_config(self):
        default_config = {
            'BLENDING_DECIDER': 'DecideBestSTI'
        }
        BlConfig().make_default_config(str(self), default_config)

    def get_decider(self, id_or_name=None):
        if id_or_name is None:
            id_or_name = BlConfig().get(str(self), 'BLENDING_DECIDER')

        decider = self.decider_list.get(str(id_or_name))
        if decider is not None:
            return decider(self.a)
        else:
            raise UserWarning('Decider not found.')

