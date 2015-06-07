import random
import random_all
from opencog.type_constructors import types
from blender_b.chooser.base_chooser import BaseChooser
from util_b.blending_util import get_incoming_node_list, sti_value_dict
from util_b.general_util import BlConfig, BlLogger

__author__ = 'DongMin Kim'


class ChooseInSTIRange(BaseChooser):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    def __str__(self):
        return 'ChooseInSTIRange'

    def __get_atoms_in_sti_range(self, atom_type, least_count, sti_min, sti_max):
        """
        Select atoms in STI range and return
        :param Type atom_type: decide the type of atoms to select
        :param int least_count: decide the minimum number of atoms to select
        :param float sti_min: min value of sti to select
        :param float sti_max: max value of sti to select
        :return:
        """
        ret = []

        a_atom_list = self.a.get_atoms_by_av(sti_min, sti_max)

        if len(a_atom_list) < least_count:
            BlLogger().log('Size of atom list is too small.')
            self.last_status = self.NOT_ENOUGH_ATOMS
            return ret

        a_found_list = filter(lambda atom: atom.is_a(atom_type), a_atom_list)
        if len(a_found_list) < least_count:
            print('Size of requested list is too small.')
            return ret

        return a_found_list

    def __atom_choose_impl(self, option):
        atom_type = option.get('atom_type')
        least_count = option.get('least_count')
        sti_min = option.get('sti_min')
        sti_max = option.get('sti_max')

        if atom_type is None:
            atom_type = types.Atom
        if least_count is None:
            least_count = 0
        if sti_min is None:
            sti_min = 1

        return self.__get_atoms_in_sti_range(
            atom_type, least_count, sti_min, sti_max
        )

    def atom_choose(self, is_use_config_file=True, option=None):
        if is_use_config_file:
            if option is None:
                option = dict()
            atom_type_str = \
                BlConfig().get('ChooseInSTIRange', 'ATOM_TYPE')
            option['atom_type'] = types().__getattribute__(atom_type_str)
            least_count_str = \
                BlConfig().get('ChooseInSTIRange', 'LEAST_COUNT')
            option['least_count'] = int(least_count_str)
            option['sti_min'] = sti_value_dict[
                BlConfig().get('ChooseInSTIRange', 'STI_MIN')
            ]
            option['sti_max'] = sti_value_dict[
                BlConfig().get('ChooseInSTIRange', 'STI_MAX')
            ]
        else:
            if option is None:
                BlLogger().log("Option was not provided.")
                self.last_status = self.PARAMETER_ERROR
                return

        return self.__atom_choose_impl(option)
