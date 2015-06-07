from opencog.type_constructors import types
from blender_b.chooser.base_chooser import BaseChooser
from util_b.general_util import BlConfig, BlLogger

__author__ = 'DongMin Kim'


class ChooseInSTIRange(BaseChooser):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(self):
        default_config = [
            ['ATOM_TYPE', 'Atom'],
            ['LEAST_COUNT', '0'],
            ['STI_MIN', '1'],
            ['STI_MAX', 'NONE'],
        ]
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
        :return:
        """
        ret = []

        a_atom_list = self.a.get_atoms_by_av(sti_min, sti_max)

        if len(a_atom_list) < least_count:
            BlLogger().log('Size of atom list is too small.')
            self.last_status = self.Status.NOT_ENOUGH_ATOMS
            return ret

        a_found_list = filter(lambda atom: atom.is_a(atom_type), a_atom_list)
        if len(a_found_list) < least_count:
            print('Size of requested list is too small.')
            return ret

        return a_found_list

    def __atom_choose_impl(self, config):
        atom_type = config.get('atom_type')
        least_count = config.get('least_count')
        sti_min = config.get('sti_min')
        sti_max = config.get('sti_max')

        if atom_type is None:
            atom_type = types.Atom
        if least_count is None:
            least_count = 0
        if sti_min is None:
            sti_min = 1

        return self.__get_atoms_in_sti_range(
            atom_type, least_count, sti_min, sti_max
        )

    def atom_choose(self, using_config_in_file=True, config=None):
        if config is None:
            config = BlConfig().get_section(str(self))

        return self.__atom_choose_impl(config)
