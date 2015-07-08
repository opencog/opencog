import random
from blending.src.chooser.chooser_finder import ChooserFinder
from blending.util.blending_util import rand_tv
from opencog.atomspace import TruthValue
from opencog.type_constructors import types, ConceptNode
from opencog.logger import log
from blending.util.link_copier import LinkCopier

__author__ = 'DongMin Kim'

class ConceptualBlending:
    def __init__(self, a):
        self.a = a

        self.link_copier_inst = LinkCopier(self.a)

        self.chooser_finder = ChooserFinder(self.a)
        self.chooser = None

    def __make_workers(self, configs):
        chooser_name = configs['atoms_chooser']
        self.chooser = self.chooser_finder.get_chooser(chooser_name)
        if self.chooser is None:
            raise UserWarning(
                "Can't find " + chooser_name + "\n" +
                "Available choosers: " + "\n" +
                str(self.chooser_finder.chooser_list)
            )

    def run(self, configs):
        log.warn("Start ConceptualBlending")

        self.__make_workers(configs)

        # Select nodes to blending.
        a_nodes = self.chooser.atom_choose({
            'atom_type': types.Node,
            'count': 2,
            'sti_min': 8
        })

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

        log.warn(str(a_blended_node.h) + " " + str(a_blended_node.name))
        log.warn("Finish ConceptualBlending")
