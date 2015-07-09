from blending.src.decider.decide_null import DecideNull
from blending.src.decider.decide_random import DecideRandom
from blending.src.decider.decide_best_sti import DecideBestSTI
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class DeciderFinder(object):
    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR

        self.deciders = {
            DecideNull.__name__: DecideNull,
            DecideRandom.__name__: DecideRandom,
            DecideBestSTI.__name__: DecideBestSTI
        }

    def get_decider(self, config_base):
        self.last_status = blending_status.IN_PROCESS

        decider = self.deciders.get(
            BlendConfig().get_str(self.a, "blending-decider", config_base)
        )
        if decider is not None:
            self.last_status = blending_status.SUCCESS
            return decider(self.a)
        else:
            self.last_status = blending_status.PARAMETER_ERROR
