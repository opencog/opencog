import random
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status
from opencog.type_constructors import types
from blending.src.chooser.base_chooser import BaseChooser
from opencog.logger import log
__author__ = 'DongMin Kim'

class ChooseAll(BaseChooser):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    def make_default_config(self):
        super(self.__class__, self).make_default_config()

    # Select atoms randomly and return
    # atom_type = decide the type of atoms to select
    # count = decide the number of atoms to select
    def __get_atoms_all(self, atom_type, count):
        ret = []

        # TODO: change to search all atomspace
        # (BlendTarget is only useful in development phase)
        self.ret = self.a.get_atoms_by_type(atom_type)

        if len(self.ret) < count:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            return

    def atom_choose_impl(self, config_base):
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
        try:
            least_count = int(least_count)
            if least_count < 0:
                raise ValueError('Least count of atom list is too small.')
        except (TypeError, ValueError):
            least_count = 0

        self.__get_atoms_all(atom_type, least_count)
