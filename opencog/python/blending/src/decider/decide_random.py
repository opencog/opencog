import random

from blending.src.decider.base_decider import BaseDecider
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class DecideRandom(BaseDecider):
    """Blending decider that deciding to blend or not by randomly choosing.

    This decider always estimates the chosen atoms are worth.
    """

    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def make_default_config(self):
        """Initialize a default config for this class."""
        super(self.__class__, self).make_default_config()

    def __decide_atoms_random(self, chosen_atoms, result_atoms_count):
        """Actual algorithm for deciding blend.

        Args:
            chosen_atoms: The atoms to decide.
            result_atoms_count: Threshold value for minimum count of decided
            atoms.
            :param chosen_atoms: list[Atom]
            :param result_atoms_count: int
        """
        if len(chosen_atoms) < result_atoms_count:
                self.last_status = blending_status.NOT_ENOUGH_ATOMS
                return

        # Generate the random number to choose atom.
        random_atom_indexes = random.sample(
            range(0, len(chosen_atoms)), result_atoms_count
        )

        self.ret = []
        for index in random_atom_indexes:
            self.ret.append(chosen_atoms[index])

    def blending_decide_impl(self, chosen_atoms, config_base):
        """Implemented factory method to deciding atoms.

        Args:
            chosen_atoms: The atoms to decide.
            config_base: A Node to save custom config.
            :param chosen_atoms: list[Atom]
            :param config_base: Atom
        """
        result_atoms_count = BlendConfig().get_int(
            self.a, "decide-result-atoms-count", config_base
        )

        self.__decide_atoms_random(chosen_atoms, result_atoms_count)
