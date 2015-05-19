__author__ = 'DongMin Kim'

import random

# To avoid unresolved reference complain in PyCharm 4.0.6
from opencog.atomspace import AtomSpace, types
from opencog.type_constructors import ConceptNode
from opencog.utilities import initialize_opencog
from opencog.logger import log

from experiment_codes import ExperimentCodes


# Perform Conceptual Blending.
class ShellBlending:
    def __init__(self):
        self.a = AtomSpace()
        initialize_opencog(self.a)

        self.experiment_codes_class = ExperimentCodes(self.a)

    def print_atomspace_for_debug(self):
        # print "Current Nodes: \n" + str(self.a.get_atoms_by_type(types.Node))
        # print "Current Links: \n" + str(self.a.get_atoms_by_type(types.Link))
        return

    def get_atomspace_for_debug(self):
        return self.a

    def call_experiment_functions(self):
        self.experiment_codes_class.execute()

    # Select atoms randomly and return
    # atom_type = decide the type of atoms to select
    # count = decide the number of atoms to select
    def _get_random_atom(self, atom_type=types.Atom, count=2):
        ret = []

        a_list = self.a.get_atoms_by_type(atom_type)
        a_list_size = a_list.__len__()

        if a_list_size < count:
            print('Size of atom list is too small')
            return ret

        a_index_list = random.sample(range(0, a_list_size), count)

        for i in a_index_list:
            ret.append(a_list[i])

        return ret

    def _random_blending(self):
        log.warn("Start RandomBlending")

        # Select nodes to blending.
        a_nodes = self._get_random_atom(types.ConceptNode, 2)

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

        self._random_blending()
        # Simulate cogserver environment.
        # Blending methods will be located in here.
        while 1:
            break

        # DEBUG: To keep program in running while view my result of coding.
        raw_input("Press enter to exit\n")


# Log will be written to opencog.log in the current directory.
log.set_level('WARN')
log.use_stdout()

# Start Conceptual Blending.
inst = ShellBlending()
inst.run()
