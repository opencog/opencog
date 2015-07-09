from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status
from opencog.type_constructors import types
from blending.src.chooser.base_chooser import BaseChooser

__author__ = 'DongMin Kim'


class ChooseInSTIRange(BaseChooser):
    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def make_default_config(self):
        super(self.__class__, self).make_default_config()
        BlendConfig().update(self.a, "choose-sti-min", "1")
        BlendConfig().update(self.a, "choose-sti-max", "None")

    def __get_atoms_in_sti_range(
            self, atom_type, least_count, sti_min, sti_max
    ):
        """
        Choose atoms within proper STI range.
        :param Type atom_type: type of atoms to choose.
        :param int least_count: minimum number of atoms to choose.
        :param float sti_min: min value of sti to choose.
        :param float sti_max: max value of sti to choose.
        :return:
        """
        ret = []

        atoms = self.a.get_atoms_by_av(sti_min, sti_max)

        if len(atoms) < least_count:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            return

        self.ret = filter(lambda atom: atom.is_a(atom_type), atoms)
        if len(self.ret) < least_count:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            return

    def atom_choose_impl(self, config_base):
        atom_type = BlendConfig().get_str(
            self.a, "choose-atom-type", config_base
        )
        least_count = BlendConfig().get_int(
            self.a, "choose-least-count", config_base
        )
        sti_min = BlendConfig().get_str(self.a, "choose-sti-min", config_base)
        sti_max = BlendConfig().get_str(self.a, "choose-sti-max", config_base)

        try:
            atom_type = types.__dict__[atom_type]
        except KeyError:
            atom_type = types.Node
        try:
            least_count = int(least_count)
            if least_count < 0:
                raise ValueError('Least count of atom list is too small.')
        except (TypeError, ValueError):
            least_count = 0
        try:
            sti_min = int(sti_min)
        except ValueError:
            sti_min = 1
        try:
            sti_max = int(sti_max)
        except ValueError:
            sti_max = None

        return self.__get_atoms_in_sti_range(
            atom_type, least_count, sti_min, sti_max
        )
