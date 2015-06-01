import random
from blender_b.chooser.base_chooser import BaseChooser
from util_b.blending_util import get_incoming_node_list

__author__ = 'DongMin Kim'

class RandomInSTIRange(BaseChooser):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    # Select atoms randomly and return
    # atom_type = decide the type of atoms to select
    # count = decide the number of atoms to select
    # sti_start = start value of sti to select
    # sti_end = end value of sti to select
    def __get_random_atom_in_sti_range(self, count=2, sti_start, sti_end):
        ret = []

        a_atom_list = self.a.xget_atoms_by_av(sti_start, sti_end)
        if len(a_atom_list) < count:
            print('Size of atom list is too small')
            return ret

        a_node_list = []
        for atom in a_atom_list:
            self.at.



            self.a.get_atoms_by_type(
            types.Node, at.get_atoms_by_av(
            10000))
        a_list_size = a_atom_list.__len__()


        a_index_list = random.sample(range(0, a_list_size), count)

        for i in a_index_list:
            ret.append(a_atom_list[i])

        return ret

    def atom_choose(self, option):
        count = option['count']
        sti_start = option['sti_start']
        sti_end = option['sti_end']
        return self.__get_random_atom_in_sti_range(count, sti_start, sti_end)


