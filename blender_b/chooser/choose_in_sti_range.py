from opencog.type_constructors import types
from blender_b.chooser.base_chooser import BaseChooser
from util_b.blending_util import sti_value_dict
from util_b.general_util import BlConfig, BlLogger

__author__ = 'DongMin Kim'


class ChooseInSTIRange(BaseChooser):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(self):
        default_config = {
            'ATOM_TYPE': 'Atom',
            'LEAST_COUNT': '0',
            'STI_MIN': '1',
            'STI_MAX': 'NONE',
        }
        BlConfig().make_default_config(str(self), default_config)

    def __get_atoms_in_sti_range(
            self, atom_type, least_count, sti_min, sti_max
    ):
        """
        Choose atoms within proper STI range.
        :param Type atom_type: type of atoms to choose.
        :param int least_count: minimum number of atoms to choose.
        :param float sti_min: min value of sti to choose.
        :param float sti_max: max value of sti to choose.
        """
        a_atom_list = self.a.get_atoms_by_av(sti_min, sti_max)

        if len(a_atom_list) < least_count:
            self.last_status = self.Status.NOT_ENOUGH_ATOMS
            raise UserWarning('Size of atom list is too small.')

        self.ret = filter(lambda atom: atom.is_a(atom_type), a_atom_list)
        if len(self.ret) < least_count:
            self.last_status = self.Status.NOT_ENOUGH_ATOMS
            raise UserWarning('Size of atom list is too small.')

    def atom_choose_impl(self, config):
        if config is None:
            config = BlConfig().get_section(str(self))

        atom_type = config.get('ATOM_TYPE')
        least_count = config.get('LEAST_COUNT')
        sti_min = config.get('STI_MIN')
        sti_max = config.get('STI_MAX')

        try:
            atom_type = types.__dict__[atom_type]
        except KeyError:
            atom_type = types.Atom
        try:
            least_count = int(least_count)
        except TypeError or ValueError:
            least_count = 0
        try:
            sti_min = sti_value_dict[sti_min]
        except KeyError:
            sti_min = 1
        try:
            sti_max = sti_value_dict[sti_max]
        except KeyError:
            sti_max = None

        return self.__get_atoms_in_sti_range(
            atom_type, least_count, sti_min, sti_max
        )
