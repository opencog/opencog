from opencog.type_constructors import types
from blender_b.chooser.base_chooser import BaseChooser
from util_b.general_util import BlAtomConfig

__author__ = 'DongMin Kim'


class ChooseAll(BaseChooser):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(self):
        super(self.__class__, self).make_default_config()

    def __get_atoms_all(self, focus_atoms, atom_type, least_count):
        """
        Choose all atoms.
        :param Type atom_type: type of atoms to choose.
        :param int least_count: minimum number of atoms to choose.
        """
        self.ret = self.a.get_atoms_by_type(atom_type, True)

        if len(self.ret) < least_count:
            self.last_status = self.Status.NOT_ENOUGH_ATOMS
            raise UserWarning('Size of atom list is too small.')

    def atom_choose_impl(self, focus_atoms, config_base):
        atom_type = BlAtomConfig().get_str(self.a, "choose-atom-type", config_base)
        least_count = BlAtomConfig().get_int(self.a, "choose-least-count", config_base)

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

        self.__get_atoms_all(focus_atoms, atom_type, least_count)
