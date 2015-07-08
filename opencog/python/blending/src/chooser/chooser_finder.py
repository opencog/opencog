from blending.src.chooser.choose_all import ChooseAll
from blending.src.chooser.choose_in_sti_range import ChooseInSTIRange
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class ChooserFinder(object):
    def __init__(self, atomspace):
        self.a = atomspace
        self.last_status = blending_status.UNKNOWN_ERROR

        self.chooser_list = {
            ChooseAll.__name__: ChooseAll,
            ChooseInSTIRange.__name__: ChooseInSTIRange
        }

    def get_chooser(self, chooser_name):
        self.last_status = blending_status.IN_PROCESS

        chooser = self.chooser_list.get(str(chooser_name))

        if chooser is not None:
            self.last_status = blending_status.SUCCESS
            return chooser(self.a)
        else:
            self.last_status = blending_status.PARAMETER_ERROR
