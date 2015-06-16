from opencog_b.python.conceptual_blending.blender.decider.decide_best_sti import \
    DecideBestSTI
from opencog_b.python.conceptual_blending.blender.decider.decide_null import \
    DecideNull
from opencog_b.python.conceptual_blending.blender.decider.decide_random import \
    DecideRandom
from opencog_b.python.conceptual_blending.util.general_util import BlendConfig

__author__ = 'DongMin Kim'


class DeciderFinder(object):
    def __init__(self, a):
        self.a = a

        self.deciders = {
            DecideNull.__name__: DecideNull,
            DecideRandom.__name__: DecideRandom,
            DecideBestSTI.__name__: DecideBestSTI
        }

    def __str__(self):
        return self.__class__.__name__

    def get_decider(self, id_or_name=None):
        if id_or_name is None:
            id_or_name = BlendConfig().get_str(self.a, "blending-decider")

        decider = self.deciders.get(str(id_or_name))
        if decider is not None:
            return decider(self.a)
        else:
            raise UserWarning('Decider not found.')
