__author__ = 'DongMin Kim'

# To avoid unresolved reference complain in PyCharm 4.0.6
from opencog.atomspace import AtomSpace, types
from opencog.utilities import initialize_opencog
from opencog.logger import log

from experiment_codes import ExperimentCodes


# Perform Conceptual Blending.
class ShellBlending:
    def __init__(self):
        self.a = AtomSpace()
        initialize_opencog(self.a)

        self.experiment_codes_class = ExperimentCodes(self.a)

    def get_atomspace_for_debug(self):
        return self.a

    def call_experiment_functions(self):
        self.experiment_codes_class.execute()

    def run(self):
        print "Start ShellBlending"
        log.info("Start ShellBlending")

        self.call_experiment_functions()
        # print "Current Nodes: \n" + str(self.a.get_atoms_by_type(types.Node))
        # print "Current Links: \n" + str(self.a.get_atoms_by_type(types.Link))

        # Simulate cogserver environment.
        # Blending methods will be located in here.
        while 1:
            break

        # DEBUG: To keep program in running while view my result of coding.
        raw_input("Press enter to exit")


# Log will be written to opencog.log in the current directory.
log.set_level('INFO')
# log.use_stdout()

# Start Conceptual Blending.
inst = ShellBlending()
inst.run()
