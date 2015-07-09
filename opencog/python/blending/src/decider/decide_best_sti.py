from bisect import bisect_left, bisect_right

from blending.src.decider.base_decider import BaseDecider
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class DecideBestSTI(BaseDecider):
    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def make_default_config(self):
        super(self.__class__, self).make_default_config()
        BlendConfig().update(self.a, "decide-sti-min", "1")
        BlendConfig().update(self.a, "decide-sti-max", "None")

    def __decide_atoms_best_sti(
            self, chosen_atoms, result_atoms_count, sti_min, sti_max
    ):
        if len(chosen_atoms) < result_atoms_count:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
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
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            raise UserWarning(
                "Result set doesn't satisfy sti value condition."
            )

        self.ret.reverse()
        self.ret = self.ret[0:result_atoms_count]

    def blending_decide_impl(self, chosen_atoms, config_base):
        result_atoms_count = BlendConfig().get_int(
            self.a, "decide-result-atoms-count", config_base
        )
        sti_min = BlendConfig().get_str(self.a, "decide-sti-min", config_base)
        sti_max = BlendConfig().get_str(self.a, "decide-sti-max", config_base)

        try:
            result_atoms_count = int(result_atoms_count)
        except (TypeError, ValueError):
            result_atoms_count = 2
        try:
            sti_min = int(sti_min)
        except ValueError:
            sti_min = 1
        try:
            sti_max = int(sti_max)
        except ValueError:
            sti_max = None

        self.__decide_atoms_best_sti(
            chosen_atoms, result_atoms_count, sti_min, sti_max
        )
