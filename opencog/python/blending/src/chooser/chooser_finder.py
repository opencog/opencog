from blending.src.chooser.choose_all import ChooseAll
from blending.src.chooser.choose_in_sti_range import ChooseInSTIRange
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class ChooserFinder(object):
    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR

        self.choosers = {
            ChooseAll.__name__: ChooseAll,
            ChooseInSTIRange.__name__: ChooseInSTIRange
        }

    def get_chooser(self, config_base):
        self.last_status = blending_status.IN_PROCESS

        chooser = self.choosers.get(
            BlendConfig().get_str(self.a, "atoms-chooser", config_base)
        )
        if chooser is not None:
            self.last_status = blending_status.SUCCESS
            return chooser(self.a)
        else:
            self.last_status = blending_status.PARAMETER_ERROR
