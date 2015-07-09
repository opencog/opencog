import random

from blending.src.decider.base_decider import BaseDecider
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class DecideRandom(BaseDecider):
    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def make_default_config(self):
        super(self.__class__, self).make_default_config()

    def __decide_atoms_random(self, chosen_atoms, result_atoms_count):
        if len(chosen_atoms) < result_atoms_count:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            raise UserWarning('Size of atom list is too small.')

        random_atom_indexes = random.sample(
            range(0, len(chosen_atoms)), result_atoms_count
        )

        self.ret = []
        for index in random_atom_indexes:
            self.ret.append(chosen_atoms[index])

    def blending_decide_impl(self, chosen_atoms, config_base):
        result_atoms_count = BlendConfig().get_int(
            self.a, "decide-result-atoms-count", config_base
        )

        try:
            result_atoms_count = int(result_atoms_count)
        except (TypeError, ValueError):
            result_atoms_count = 2

        self.__decide_atoms_random(chosen_atoms, result_atoms_count)
