__author__ = 'DongMin Kim'

import random

from opencog.atomspace import *
from opencog.type_constructors import *
from opencog.logger import *

from base_blender import *

import blending_util
from blending_util import *


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

    def _get_valuable_blend_target_list(self):
        ret = self._get_incoming_dst_atom(
            self.a_blend_target, types.ConceptNode)
        return ret

    # Select atoms randomly and return
    # atom_type = decide the type of atoms to select
    # count = decide the number of atoms to select
    def _get_random_atom(self, count=2):
        ret = []

        # TODO: change to search all atomspace
        # (BlendTarget is only useful in development phase)
        a_atom_list = self._get_valuable_blend_target_list()
        a_list_size = a_atom_list.__len__()

        if a_list_size < count:
            print('Size of atom list is too small')
            return ret

        a_index_list = random.sample(range(0, a_list_size), count)

        for i in a_index_list:
            ret.append(a_atom_list[i])

        return ret

    def _correct_strength_of_links(self, found_link, exist_link):
        found_s = found_link.tv.mean
        found_c = found_link.tv.confidence
        exist_s = exist_link.tv.mean
        exist_c = exist_link.tv.confidence

        # sC = (cA sA + cB sB) / (cA + cB)
        # https://groups.google.com/forum/#!topic/opencog/fa5c4yE8YdU
        new_strength = \
            ((found_c * found_s) + (exist_c * exist_s)) \
            / (found_c + exist_c)

        # TODO: Currently, conflicting confidence value for new blended node
        # is just average of old value.
        new_confidence = (found_c + exist_c) / 2

        return TruthValue(new_strength, new_confidence)

    def _copy_link_from_exist_node_list(self, src_node_list, dst_node):
        """
        valuable_node_list = self._get_valuable_blend_target_list()
        valuable_node_handle_set = set()
        for valuable_node in valuable_node_list:
            valuable_node_handle_set.add(valuable_node.h.value())

        dst_node_link_out_handle_set = []

        for src_node in src_node_list:
            src_node_link_list = self.a.get_atoms_by_target_atom(
                types.Link, src_node
            )

            for src_node_link in src_node_link_list:
                src_node_link_out_list = self.a.get_outgoing(src_node_link.h)

                for src_node_link_out in src_node_link_out_list:
                    if src_node_link_out.h.value() in valuable_node_handle_set:
                        dst_node_link_out_handle_set. \
                            append((src_node_link_out, src_node_link))

        dst_node_list = list(
            dst_node_link_out_handle_set & valuable_node_handle_set
        )

        for node in src_node_list:
            src_link_list = self.a.get_atoms_by_target_atom(types.Link,
                                                            src_node)
            dst_link_list = self.a.get_atoms_by_target_atom(types.Link,
                                                            dst_node)

            dst_out_dict = dict()
            for dst_link in dst_link_list:
                dst_out_list = self.a.get_outgoing(dst_link.h)
                dst_out_handle_set = set()

                for dst_out in dst_out_list:
                    dst_out_handle_set.add(dst_out.h.value())

                dst_out_handle_set &= valuable_node_handle_set

                for dst_out_handle in list(dst_out_handle_set):
                    dst_out_dict[dst_out_handle] = self.a.g

            for src_link in src_link_list:
                src_out_list = self.a.get_outgoing(src_link.h)
                src_out_set = set()

                for src_out in src_out_list:
                    src_out_set.add(src_out.h.value())

                src_out_set &= valuable_node_handle_set

                for src_out in list(src_out_set):
                    corrected_link_tv = src_out.tv

                    if src_out.h.value() in dst_out_dict:
                        corrected_link_tv = \
                            self._correct_strength_of_links(
                                src_link,
                                dst_out_dict.get(src_out.h.value())
                            )

                    if (src_out.h != src_node.h) and \
                            (src_out.h != dst_node.h):
                        self.a.add_link(
                            src_link.t,
                            [src_out, dst_node],
                            corrected_link_tv
                        )
        """
        return None

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

        print "New node: " + a_blended_node.name

        # Link with blend target.
        self.a.add_link(
            types.MemberLink,
            [a_blended_node, self.a_blend_target],
            blend_target_link_tv
        )

        # Make the links between exist nodes and newly blended node.
        self._copy_link_from_exist_node_list([a_node_0, a_node_1],
                                             a_blended_node)

        # Correct some attribute value of new links.
        # self._correct_strength_of_links(a_blended_node)

        # Detect and improve conflict links in newly blended node.
        # - Do nothing.

        log.warn(str(a_blended_node.h) + str(a_blended_node.name))
        log.warn("Finish RandomBlending")
