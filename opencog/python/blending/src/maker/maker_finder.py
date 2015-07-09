from blending.src.maker.make_simple import MakeSimple
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class MakerFinder(object):
    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR

        self.makers = {
            MakeSimple.__name__: MakeSimple
        }

    def get_maker(self, config_base):
        self.last_status = blending_status.IN_PROCESS

        maker = self.makers.get(
            BlendConfig().get_str(self.a, "new-blend-atom-maker", config_base)
        )
        if maker is not None:
            self.last_status = blending_status.SUCCESS
            return maker(self.a)
        else:
            self.last_status = blending_status.PARAMETER_ERROR
