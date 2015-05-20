__author__ = 'DongMin Kim'

from opencog.atomspace import *
from opencog.type_constructors import *
from opencog.utilities import *
from opencog.logger import *

from blender.random_blender import *
import blending_util

from experiment_codes import ExperimentCodes

# Perform Conceptual Blending.
class ShellBlending:
    def __init__(self):
        self.a = AtomSpace()
        initialize_opencog(self.a)
        self._make_blend_target_for_debug()

        self.experiment_codes_inst = ExperimentCodes(self.a)
        self.random_blending_inst = RandomBlender(self.a)

    def __del__(self):
        self._delete_blend_target_for_debug()

    def print_atomspace_for_debug(self):
        print "Current Nodes: \n" + str(self.a.get_atoms_by_type(types.Node))
        print "Current Links: \n" + str(self.a.get_atoms_by_type(types.Link))

    def get_atomspace_for_debug(self):
        return self.a

    # DEBUG: Make temporary concept - To define which node is target to blend.
    def _make_blend_target_for_debug(self):
        self.a_blend_target = ConceptNode(
            blending_util.BLEND_TARGET_NODE_NAME,
        )

    def _delete_blend_target_for_debug(self):
        # l_link_list = self.a.get_incoming(self.a_blend_target.h)
        l_link_list = self.a.get_atoms_by_target_atom(
            types.Link, self.a_blend_target
        )

        for link in l_link_list:
            self.a.remove(link)

        self.a.remove(self.a_blend_target)

    def call_experiment_functions(self):
        self.experiment_codes_inst.execute()

    def run(self):
        log.warn("Start ShellBlending")

        self.call_experiment_functions()

        # Simulate cogserver environment.
        # Blending methods will be located in here.
        while 1:
            self.random_blending_inst.blend()
            is_stop = raw_input("Input q to stop, or continue.\n")
            if is_stop == 'q' or is_stop == 'Q':
                break

# Log will be written to opencog.log in the current directory.
log.set_level('WARN')
log.use_stdout()

# Start Conceptual Blending.
inst = ShellBlending()
inst.run()
inst.__del__()

# DEBUG: To keep program in running while view my result of coding.
raw_input("Press enter to exit\n")
