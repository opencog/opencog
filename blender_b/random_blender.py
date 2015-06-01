# coding=utf-8
from util_b.general_util import BlendingLoggerForDebug

__author__ = 'DongMin Kim'

from base_blender import *
from util_b.blending_util import *
from util_b.link_copier import *


class RandomBlender(BaseBlender):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

        self.chooser = self.chooser = \
            self.chooser_factory.get_chooser("RandomInSTIRange")

        self.link_copier_inst = LinkCopier(self.a)

    def __str__(self):
        return 'RandomBlender'

    def __atom_choose(self):
        chooser_option = {
            'atom_type': types.Node,
            'count': 2,
            'sti_min': 8
        }
        return self.chooser.atom_choose(chooser_option)

    def get_last_status(self):
        return self.last_status

    def blend(self):
        BlendingLoggerForDebug().log("Start RandomBlending")

        # Select nodes to blending.
        a_nodes = self.__atom_choose()

        # Decide whether or not to execute blending and prepare.
        # - Do nothing.

        # Check the conflict links in each node and remove
        # - Do nothing.

        # Make the blended node.
        a_node_0 = a_nodes[0]
        a_node_1 = a_nodes[1]
        a_blended_node = ConceptNode(
            '(' +
            str(a_node_0.name) + '_' + str(a_node_1.name) +
            ')',
            rand_tv()
        )

        # Make the links between exist nodes and newly blended node.
        # Correct some attribute value of new links.
        self.link_copier_inst.copy_all_link_to_new_node(
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

        # DEBUG: Link with blend target.
        self.a.add_link(
            types.MemberLink,
            [a_blended_node, self.a_blend_target],
            blend_target_link_tv
        )

        # DEBUG: Link with Blended Space.
        a_blended_space = \
            self.a.get_atoms_by_name(
                types.Atom,
                "BlendedSpace"
            )[0]

        self.a.add_link(
            types.MemberLink,
            [a_blended_node, a_blended_space],
            blend_target_link_tv
        )

        BlendingLoggerForDebug().log(str(a_blended_node.h) + " " + str(a_blended_node.name))
        BlendingLoggerForDebug().log("Finish RandomBlending")

        return 0
