import random
from opencog.type_constructors import types
from blending.src.chooser.base_chooser import BaseChooser

__author__ = 'DongMin Kim'

class ChooseRandomAll(BaseChooser):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    def __str__(self):
        return 'ChooseRandomAll'

    # Select atoms randomly and return
    # atom_type = decide the type of atoms to select
    # count = decide the number of atoms to select
    def __get_random_atoms(
            self, atom_type=types.Node, count=2
    ):
        ret = []

        # TODO: change to search all atomspace
        # (BlendTarget is only useful in development phase)
        a_atom_list = self.a.get_atoms_by_type(atom_type)
        a_list_size = a_atom_list.__len__()

        if a_list_size < count:
            print('Size of atom list is too small')
            return ret

        a_index_list = random.sample(range(0, a_list_size), count)

        for i in a_index_list:
            ret.append(a_atom_list[i])

        return ret

    def atom_choose(self, option):
        atom_type = option.get('atom_type')
        count = option['count']
        return self.__get_random_atoms(atom_type, count)
