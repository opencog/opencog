from bisect import bisect_left, bisect_right
from blender_b.decider.base_decider import BaseDecider
from util_b.blending_util import sti_value_dict
from util_b.general_util import BlAtomConfig

__author__ = 'DongMin Kim'


class DecideBestSTI(BaseDecider):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(self):
        super(self.__class__, self).make_default_config()
        BlAtomConfig().add(self.a, "decide-sti-min", "IMPORTANT")
        BlAtomConfig().add(self.a, "decide-sti-max", "NONE")

    def __decide_atoms_best_sti(
            self, chosen_atoms,
            result_atoms_count, sti_min, sti_max
    ):
        """
        Choose all atoms.
        :param List chosen_atoms: atoms list to decide.
        :param int result_atoms_count: maximum number of atoms to blend.
        :param float sti_min: min value of sti to choose.
        :param float sti_max: max value of sti to choose.
        """

        if len(chosen_atoms) < result_atoms_count:
            self.last_status = self.Status.NOT_ENOUGH_ATOMS
            raise UserWarning('Size of atom list is too small.')

        self.ret = sorted(
            chosen_atoms,
            key=lambda chosen_atom: chosen_atom.av['sti']
        )
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
        self.ret = self.ret[0:result_atoms_count]

    def blending_decide_impl(self, chosen_atoms, config_base):
        result_atoms_count = BlAtomConfig().get_int(
            self.a, "decide-result-atoms-count", config_base
        )
        sti_min = BlAtomConfig().get_str(self.a, "decide-sti-min", config_base)
        sti_max = BlAtomConfig().get_str(self.a, "decide-sti-max", config_base)

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
            chosen_atoms,
            result_atoms_count, sti_min, sti_max
        )
