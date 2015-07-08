import random
from opencog.atomspace import Handle, TruthValue
from opencog.type_constructors import types, ConceptNode
from opencog.logger import *
from blending.src.base_blender import Blender


__author__ = 'DongMin Kim'

class RandomBlender(Blender):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    # Select atoms randomly and return
    # atom_type = decide the type of atoms to select
    # count = decide the number of atoms to select
    def _get_random_atom(self, count=2):
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

    def _correct_strength_of_links(
            self, src_link_dict, dst_link_dict, non_exclusive_link_set
    ):
        for link in non_exclusive_link_set:
            src_link = self.a[Handle(src_link_dict[link])]
            dst_link = self.a[Handle(dst_link_dict[link])]

            found_s = src_link.tv.mean
            found_c = src_link.tv.confidence
            exist_s = dst_link.tv.mean
            exist_c = dst_link.tv.confidence

            # sC = (cA sA + cB sB) / (cA + cB)
            # https://groups.google.com/forum/#!topic/opencog/fa5c4yE8YdU
            new_strength = \
                ((found_c * found_s) + (exist_c * exist_s)) \
                / (found_c + exist_c)

            # TODO: Currently, conflicting confidence value for new blended node
            # is just average of old value.
            new_confidence = (found_c + exist_c) / 2

            dst_link.tv = TruthValue(new_strength, new_confidence)

    def _get_linktype_linktarget_pair_dict(self, link_list, src_node):
        ret_dict = dict()
        for link in link_list:
            xget_link_node = self.a.xget_outgoing(link.h)
            for link_node in xget_link_node:
                if link_node.h != src_node.h:
                    type_target_pair = (link.t, link_node.h.value())
                    ret_dict[type_target_pair] = link.h.value()

        return ret_dict

    def _copy_all_link_to_new_node(self, src_node_list, dst_node):
        dst_link_list = self.a.get_atoms_by_target_atom(types.Link, dst_node)

        dst_link_dict = self._get_linktype_linktarget_pair_dict(
            dst_link_list, dst_node
        )
        dst_link_set = set(dst_link_dict.keys())

        for src_node in src_node_list:
            src_link_list = self.a.get_atoms_by_target_atom(
                types.Link, src_node
            )
            src_link_dict = self._get_linktype_linktarget_pair_dict(
                src_link_list, src_node
            )
            src_link_set = set(src_link_dict.keys())

            exclusive_link_set = src_link_set - dst_link_set
            non_exclusive_link_set = src_link_set & dst_link_set

            # Add new link to target node.
            for link_pair in exclusive_link_set:
                src_link = self.a[Handle(src_link_dict[link_pair])]
                src_link_type = link_pair[0]
                src_link_node = self.a[Handle(link_pair[1])]
                new_link = self.a.add_link(
                    src_link_type,
                    [dst_node, src_link_node],
                    src_link.tv
                )
                dst_link_dict[link_pair] = new_link.h.value()
                dst_link_set.add(link_pair)

            # Correct conflict link value in target node.
            self._correct_strength_of_links(
                src_link_dict, dst_link_dict, non_exclusive_link_set
            )

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
            '-' +
            str(a_node_1.name)
        )

        # Make the links between source nodes and newly blended node.
        # TODO: Change to make with proper reason, not make in every blending.
        self.a.add_link(
            types.AssociativeLink,
            [a_node_0, a_blended_node],
            TruthValue(0.7, 0.6)
        )
        self.a.add_link(
            types.AssociativeLink,
            [a_node_1, a_blended_node],
            TruthValue(0.7, 0.6)
        )

        # Make the links between exist nodes and newly blended node.
        # Correct some attribute value of new links.
        self._copy_all_link_to_new_node([a_node_0, a_node_1], a_blended_node)

        # Detect and improve conflict links in newly blended node.
        # - Do nothing.

        log.warn(str(a_blended_node.h) + " " + str(a_blended_node.name))
        log.warn("Finish RandomBlending")
