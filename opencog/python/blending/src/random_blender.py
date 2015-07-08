import random
from blending.util.blending_util import rand_tv

from opencog.atomspace import TruthValue
from opencog.type_constructors import types, ConceptNode
from opencog.logger import *
from blending.src.base_blender import BaseBlender
from blending.util.link_copier import LinkCopier

__author__ = 'DongMin Kim'

class RandomBlender(BaseBlender):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)
        self.link_copier_class = LinkCopier(self.a)

    def __str__(self):
        return 'RandomBlender'

    def get_last_status(self):
        return self.last_status

    # Select atoms randomly and return
    # atom_type = decide the type of atoms to select
    # count = decide the number of atoms to select
    def __get_random_atom(self, count=2):
        ret = []

        # TODO: change to search all atomspace
        a_atom_list = self.a.get_atoms_by_type(types.Node)
        a_list_size = a_atom_list.__len__()

        if a_list_size < count:
            print('Size of atom list is too small')
            return ret

        a_index_list = random.sample(range(0, a_list_size), count)

        for i in a_index_list:
            ret.append(a_atom_list[i])

        return ret

    def blend(self):
        # Select nodes to blending.
        a_nodes = self.__get_random_atom(2)

        # Decide whether or not to execute blending and prepare.
        # - Do nothing.

        # Check the conflict links in each node and remove
        # - Do nothing.

        # Make the blended node.
        a_node_0 = a_nodes[0]
        a_node_1 = a_nodes[1]
        a_blended_node = ConceptNode(
            str(a_node_0.name) +
            '-' +
            str(a_node_1.name),
            rand_tv()
        )

        # Make the links between exist nodes and newly blended node.
        # Correct some attribute value of new links.
        self.link_copier_class.copy_all_link_to_new_node(
            [a_node_0, a_node_1],
            a_blended_node
        )

        # Make the links between source nodes and newly blended node.
        # TODO: Change to make with proper reason, not make in every blending.
        self.a.add_link(
            types.AssociativeLink,
            [a_node_0, a_blended_node],
            rand_tv()
        )
        self.a.add_link(
            types.AssociativeLink,
            [a_node_1, a_blended_node],
            rand_tv()
        )

        # Detect and improve conflict links in newly blended node.
        # - Do nothing.

        log.warn(str(a_blended_node.h) + " " + str(a_blended_node.name))
