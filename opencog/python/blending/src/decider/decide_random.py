import random
from blending.src.decider.base_decider import BaseDecider
from blending.util.blend_config import BlendConfig
__author__ = 'DongMin Kim'


class DecideRandom(BaseDecider):
    def __init__(self, a):
        super(self.__class__, self).__init__(a)
        self.ret = list()

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(self):
        super(self.__class__, self).make_default_config()

    def __decide_atoms_random(self, chosen_atoms, result_atoms_count):
        """
        Choose all atoms.
        :param List chosen_atoms: atoms list to decide.
        :param int result_atoms_count: maximum number of atoms to blend.
        """

        if len(chosen_atoms) < result_atoms_count:
            self.last_status = self.Status.NOT_ENOUGH_ATOMS
            raise UserWarning('Size of atom list is too small.')

        random_atom_indexes = random.sample(
            range(0, len(chosen_atoms)), result_atoms_count
        )

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
