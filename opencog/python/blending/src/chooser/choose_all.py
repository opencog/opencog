import random
from blending.util.blending_error import blending_status
from opencog.type_constructors import types
from blending.src.chooser.base_chooser import BaseChooser
from opencog.logger import log
__author__ = 'DongMin Kim'

class ChooseAll(BaseChooser):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    def make_default_config(self):
        self.default_config = {
            'choose-atom-type': 'Node',
            'choose-least-count': '2'
        }

    # Select atoms randomly and return
    # atom_type = decide the type of atoms to select
    # count = decide the number of atoms to select
    def __get_atoms_all(
            self, atom_type=types.Node, count=2
    ):
        ret = []

        # TODO: change to search all atomspace
        # (BlendTarget is only useful in development phase)
        atoms = self.a.get_atoms_by_type(atom_type)

        if len(atoms) < count:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            return ret

        a_index_list = random.sample(range(0, len(atoms)), count)

        for i in a_index_list:
            ret.append(atoms[i])

        return ret

    def __atom_choose_impl(self, config):
        atom_type = config.get('choose-atom-type')
        least_count = config.get('choose-least-count')

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

        return self.__get_atoms_all(atom_type, least_count)

    def atom_choose(self, config):
        if config is None:
            config = self.default_config

        return self.__atom_choose_impl(config)
