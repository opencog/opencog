from bisect import bisect_left, bisect_right
import random
from opencog.type_constructors import types
from blender_b.chooser.base_chooser import BaseChooser
from blender_b.decider.base_decider import BaseDecider
from util_b.blending_util import sti_value_dict
from util_b.general_util import BlLogger, BlConfig

__author__ = 'DongMin Kim'


class DecideBestSTI(BaseDecider):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(self):
        default_config = {
            'RESULT_ATOMS_COUNT': '2',
            'STI_MIN': '1',
            'STI_MAX': 'NONE'
        }
        BlConfig().make_default_config(str(self), default_config)

    def __decide_atoms_best_sti(
            self, a_chosen_atoms_list,
            result_atoms_count, sti_min, sti_max
    ):
        """
        Choose all atoms.
        :param List a_chosen_atoms_list: atoms list to decide.
        :param int result_atoms_count: maximum number of atoms to blend.
        :param float sti_min: min value of sti to choose.
        :param float sti_max: max value of sti to choose.
        """

        if len(a_chosen_atoms_list) < result_atoms_count:
            self.last_status = self.Status.NOT_ENOUGH_ATOMS
            raise UserWarning('Size of atom list is too small.')

        self.ret = sorted(a_chosen_atoms_list, key=lambda atom: atom.av['sti'])
        keys = [atom.av['sti'] for atom in self.ret]

        l_index = bisect_left(keys, sti_min)
        r_index = None if sti_max is None else bisect_right(keys, sti_max)

        self.ret = self.ret[l_index: r_index]
        if len(self.ret) < result_atoms_count:
            self.last_status = self.Status.NOT_ENOUGH_ATOMS
            raise UserWarning(
                "Result set doesn't satisfy sti value condition."
            )

        self.ret.reverse()

    def blending_decide_impl(self, a_chosen_atoms_list, config):
        if config is None:
            config = BlConfig().get_section(str(self))

        result_atoms_count = config.get('RESULT_ATOMS_COUNT')
        sti_min = config.get('STI_MIN')
        sti_max = config.get('STI_MAX')

        try:
            result_atoms_count = int(result_atoms_count)
        except (TypeError, ValueError):
            result_atoms_count = 2
        try:
            sti_min = sti_value_dict[sti_min]
            sti_min = int(sti_min)
        except (KeyError, TypeError):
            sti_min = 1
        try:
            sti_max = sti_value_dict[sti_max]
        except KeyError:
            # Including if config was 'None'
            sti_max = None

        self.__decide_atoms_best_sti(
            a_chosen_atoms_list,
            result_atoms_count, sti_min, sti_max
        )
