__author__ = 'DongMin Kim'

import random

from opencog.atomspace import *
from opencog.type_constructors import *
from opencog.utilities import *
from opencog.logger import *

from experiment_codes import ExperimentCodes


# Perform Conceptual Blending.
class ShellBlending:
    def __init__(self):
        self.a = AtomSpace()
        initialize_opencog(self.a)

        self.experiment_codes_class = ExperimentCodes(self.a)

    def print_atomspace_for_debug(self):
        print "Current Nodes: \n" + str(self.a.get_atoms_by_type(types.Node))
        print "Current Links: \n" + str(self.a.get_atoms_by_type(types.Link))

    def get_atomspace_for_debug(self):
        return self.a

    def _delete_blend_target_for_debug(self):
        a_blend_target = \
            self.a.get_atoms_by_name(types.Atom, "BlendTarget")[0]

        l_link_list = self.a.get_incoming(a_blend_target.h)

        for link in l_link_list:
            self.a.remove(link)

        self.a.remove(a_blend_target)

    def call_experiment_functions(self):
        self.experiment_codes_class.execute()

    # Select atoms which are connected to specific atom.
    def _get_incoming_dst_atom(self, atom, atom_type=types.Atom):
        ret = []

        l_link_list = self.a.get_incoming(atom.h)

        for link in l_link_list:
            if (link.out.__len__() > 0) and (link.out[0].t == atom_type):
                ret.append(link.out[0])

        return ret

    # Select atoms randomly and return
    # atom_type = decide the type of atoms to select
    # count = decide the number of atoms to select
    def _get_random_atom(self, count=2):
        ret = []

        # TODO: change to search all atomspace
        # (BlendTarget is only useful in development phase)
        a_blend_target = \
            self.a.get_atoms_by_name(types.Atom, "BlendTarget")[0]

        a_atom_list = \
            self._get_incoming_dst_atom(a_blend_target, types.ConceptNode)
        a_list_size = a_atom_list.__len__()

        if a_list_size < count:
            print('Size of atom list is too small')
            return ret

        a_index_list = random.sample(range(0, a_list_size), count)

        for i in a_index_list:
            ret.append(a_atom_list[i])

        return ret

    def _random_blending(self):
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
            [
                a_blended_node,
                self.a.get_atoms_by_name(types.Atom, "BlendTarget")[0]
            ]
        )

        # Make the links between exist nodes and newly blended node.
        # - Do nothing.

        # Correct some attribute value of new links.
        # - Do nothing.

        # Detect and improve conflict links in newly blended node.
        # - Do nothing.

        log.warn(str(a_blended_node))
        log.warn("Finish RandomBlending")

    def run(self):
        log.warn("Start ShellBlending")

        self.call_experiment_functions()

        # Simulate cogserver environment.
        # Blending methods will be located in here.
        while 1:
            self._random_blending()
            is_stop = raw_input("Input q to stop, or continue.\n")
            if is_stop == 'q' or is_stop == 'Q':
                break

        # DEBUG: To keep program in running while view my result of coding.
        self._delete_blend_target_for_debug()
        raw_input("Press enter to exit\n")


# Log will be written to opencog.log in the current directory.
log.set_level('WARN')
log.use_stdout()

# Start Conceptual Blending.
inst = ShellBlending()
inst.run()
