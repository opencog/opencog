import random
from opencog.type_constructors import types
from blender_b.chooser.base_chooser import BaseChooser
from blender_b.decider.base_decider import BaseDecider
from util_b.general_util import BlLogger, BlConfig

__author__ = 'DongMin Kim'


class DecideRandom(BaseDecider):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(self):
        default_config = {
            'RESULT_ATOMS_COUNT': '2'
        }
        BlConfig().make_default_config(str(self), default_config)

    def __decide_atoms_random(self, a_chosen_atoms_list, result_atoms_count):
        """
        Choose all atoms.
        :param List a_chosen_atoms_list: atoms list to decide.
        :param int result_atoms_count: maximum number of atoms to blend.
        """

        if len(a_chosen_atoms_list) < result_atoms_count:
            self.last_status = self.Status.NOT_ENOUGH_ATOMS
            raise UserWarning('Size of atom list is too small.')

        a_index_list = random.sample(
            range(0, len(a_chosen_atoms_list)), result_atoms_count
        )

        for index in a_index_list:
            self.ret.extend(a_chosen_atoms_list[index])

    def blending_decide_impl(self, a_chosen_atoms_list, config):
        if config is None:
            config = BlConfig().get_section(str(self))

        result_atoms_count = config.get('RESULT_ATOMS_COUNT')

        try:
            result_atoms_count = int(result_atoms_count)
        except (TypeError, ValueError):
            result_atoms_count = 2

        self.__decide_atoms_random(a_chosen_atoms_list, result_atoms_count)
