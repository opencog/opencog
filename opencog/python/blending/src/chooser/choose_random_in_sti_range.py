import random
from opencog.type_constructors import types
from blending.src.chooser.base_chooser import BaseChooser

__author__ = 'DongMin Kim'

class ChooseRandomInSTIRange(BaseChooser):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    def __str__(self):
        return 'ChooseRandomInSTIRange'

    # Select atoms randomly and return
    # atom_type = decide the type of atoms to select
    # count = decide the number of atoms to select
    # sti_min = min value of sti to select
    # sti_max = max value of sti to select
    def __get_randoms_atom_in_sti_range(
            self, atom_type, count,
            sti_min, sti_max=None
    ):
        ret = []

        a_atom_list = self.a.get_atoms_by_type(atom_type, True)
        a_atom_in_sti_list = self.a.get_atoms_by_av(sti_min, sti_max)
        if (len(a_atom_list) < count) or \
                (len(a_atom_in_sti_list) < count):
            print('Size of atom list is too small')
            return ret

        # TODO: this is very inefficient method to check type of atoms.
        # Why AtomSpace doesn't provide API like 'is_type(type, atom)'?
        a_atom_set = set(a_atom_list)
        a_atom_in_sti_list = set(a_atom_in_sti_list)

        a_found_set = a_atom_set & a_atom_in_sti_list
        if len(a_atom_set) < count:
            print('Size of requested list is too small')
            return ret

        a_found_list = list(a_found_set)

        a_index_list = random.sample(range(0, len(a_found_list)), count)
        for i in a_index_list:
            ret.append(a_found_list[i])

        return ret

    def atom_choose(self, option):
        atom_type = option.get('atom_type')
        count = option.get('count')
        sti_min = option.get('sti_min')
        sti_max = option.get('sti_max')
        return self.__get_randoms_atom_in_sti_range(
            atom_type, count,
            sti_min, sti_max
        )
