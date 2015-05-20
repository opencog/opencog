__author__ = 'DongMin Kim'

import random

from opencog.atomspace import *
from opencog.type_constructors import *
from opencog.logger import *

from base_blender import *

class RandomBlender(BaseBlender):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    # Select atoms which are connected to specific atom.
    def _get_incoming_dst_atom(self, atom, atom_type=types.Atom):
        ret = []

        l_link_list = self.a.get_atoms_by_target_atom(types.Atom, atom)

        for link in l_link_list:
            for node in link.out:
                if (node.t == atom_type) and \
                        (node.h != self.a_blend_target.h):
                    ret.append(link.out[0])

        return ret

    # Select atoms randomly and return
    # atom_type = decide the type of atoms to select
    # count = decide the number of atoms to select
    def _get_random_atom(self, count=2):
        ret = []

        # TODO: change to search all atomspace
        # (BlendTarget is only useful in development phase)
        a_atom_list = \
            self._get_incoming_dst_atom(self.a_blend_target, types.ConceptNode)
        a_list_size = a_atom_list.__len__()

        if a_list_size < count:
            print('Size of atom list is too small')
            return ret

        a_index_list = random.sample(range(0, a_list_size), count)

        for i in a_index_list:
            ret.append(a_atom_list[i])

        return ret

    def _copy_link_from_exist_node(self, src_node_list, dst_node):
        for node in src_node_list:
            link_list = self.a.get_atoms_by_target_atom(types.Link, node)
            for link in link_list:
                out_list = self.a.get_outgoing(link.h)
                for out in out_list:
                    if (out.name != node.name) and \
                            (out.name != dst_node.name):
                        self.a.add_link(link.t, [out, dst_node], link.tv)

    def _correct_strength_of_links(self, dst_node):
        link_list = self.a.get_atoms_by_target_atom(types.Link, dst_node)
        # TODO: check truth value bug?

    def blend(self):
        log.warn("Start RandomBlending")

        # Select nodes to blending.
        a_nodes = self._get_random_atom(2)

        # Decide whether or not to execute blending and prepare.
        # - Do nothing.

        # Check the conflict links in each node and remove
        # - Do nothing.

        # Make the blended node.
        a_node_0 = a_nodes[0]
        a_node_1 = a_nodes[1]
        a_blended_node = ConceptNode(
            str(a_node_0.name) +
            str(a_node_1.name)
        )

        # Link with blend target.
        self.a.add_link(
            types.MemberLink,
            [a_blended_node, self.a_blend_target]
        )

        # Make the links between exist nodes and newly blended node.
        self._copy_link_from_exist_node([a_node_0, a_node_1], a_blended_node)

        # Correct some attribute value of new links.
        self._correct_strength_of_links(a_blended_node)

        # Detect and improve conflict links in newly blended node.
        # - Do nothing.

        log.warn(str(a_blended_node.h)+str(a_blended_node.name))
        log.warn("Finish RandomBlending")
