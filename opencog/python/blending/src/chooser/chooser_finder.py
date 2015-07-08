from blending.src.chooser.choose_random_all import ChooseRandomAll
from blending.src.chooser.choose_random_in_sti_range import \
    ChooseRandomInSTIRange
from opencog.logger import log

__author__ = 'DongMin Kim'


class ChooserFinder(object):
    def __init__(self, atomspace):
        self.a = atomspace

        self.chooser_list = [
            ChooseRandomAll,
            ChooseRandomInSTIRange
        ]

    def get_chooser(self, chooser_name):
        for chooser in self.chooser_list:
            if str(chooser).find(chooser_name) != -1:
                return chooser(self.a)
