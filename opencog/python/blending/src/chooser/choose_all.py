from opencog.type_constructors import types

from blending.src.chooser.base_chooser import BaseChooser
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class ChooseAll(BaseChooser):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    def make_default_config(self):
        super(self.__class__, self).make_default_config()

    def __get_atoms_all(self, focus_atoms, atom_type, least_count):
        self.ret = filter(lambda atom: atom.is_a(atom_type), focus_atoms)

        if len(self.ret) < least_count:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            return

    def atom_choose_impl(self, focus_atoms, config_base):
        focus_atoms = self.a.get_atoms_by_type(types.Atom) \
            if len(focus_atoms) is 0 \
            else focus_atoms

        atom_type = BlendConfig().get_str(
            self.a, "choose-atom-type", config_base
        )
        least_count = BlendConfig().get_int(
            self.a, "choose-least-count", config_base
        )

        try:
            atom_type = types.__dict__[atom_type]
        except KeyError:
            atom_type = types.Node

        if least_count < 0:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            return

        self.__get_atoms_all(focus_atoms, atom_type, least_count)
