from opencog.type_constructors import types
from blender_b.chooser.base_chooser import BaseChooser
from util_b.general_util import BlLogger, BlConfig

__author__ = 'DongMin Kim'


class ChooseAll(BaseChooser):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(self):
        default_config = [
            ['ATOM_TYPE', 'Atom'],
            ['LEAST_COUNT', '0'],
        ]
        BlConfig().make_default_config(str(self), default_config)

    def __get_atoms_all(self, atom_type, least_count):
        """
        Choose all atoms.
        :param Type atom_type: type of atoms to choose.
        :param int least_count: minimum number of atoms to choose.
        :return:
        """
        ret = []

        a_atom_list = self.a.get_atoms_by_type(atom_type, True)

        if len(a_atom_list) < least_count:
            BlLogger().log('Size of atom list is too small.')
            self.last_status = self.Status.NOT_ENOUGH_ATOMS
            return ret

        return ret

    def __atom_choose_impl(self, config):
        atom_type = config.get('atom_type')
        least_count = config.get('least_count')

        if atom_type is None:
            atom_type = types.Atom
        if least_count is None:
            least_count = 0

        return self.__get_atoms_all(atom_type, least_count)

    def atom_choose(self, config=None):
        if config is None:
            config = BlConfig().get_section(str(self))

        return self.__atom_choose_impl(config)
