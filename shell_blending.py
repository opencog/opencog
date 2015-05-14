__author__ = 'DongMin Kim'

"""
# Import * will be deleted in later
from opencog import *
from opencog.atomspace import *
from opencog.type_constructors import *
from opencog.utilities import *
from opencog.logger import *
"""

# To avoid unresolved reference complain in PyCharm 4.0.6
from opencog.atomspace import AtomSpace, TruthValue, types
from opencog.type_constructors import ConceptNode, UnorderedLink
from opencog.utilities import initialize_opencog
from opencog.logger import log


# Make several test cases for debug.
class TestCaseMaker:
    def __init__(self, atomspace):
        self.a = atomspace
        self.atom_list_for_debug = []
        self.link_list_for_debug = []

    def _make_test_case(self):
        # Paul & Sally example in the book 'The Way We Think'
        a1 = ConceptNode("Paul", TruthValue(0.9, 0.8))
        a2 = ConceptNode("Sally", TruthValue(0.9, 0.8))
        l1 = UnorderedLink([a1, a2], TruthValue(1.0, 0.8))
        self.atom_list_for_debug.extend([a1, a2])
        self.link_list_for_debug.append(l1)

    def make(self):
        self._make_test_case()


# Perform Conceptual Blending.
class ShellBlending:
    def __init__(self):
        self.a = AtomSpace()
        initialize_opencog(self.a)

        self.test_case_maker = TestCaseMaker(self.a)
        self.test_case_maker.make()

    def run(self):
        print "Start ShellBlending"
        log.info("Start ShellBlending")
        print "Current Nodes: \n" + str(self.a.get_atoms_by_type(types.Node))
        print "Current Links: \n" + str(self.a.get_atoms_by_type(types.Link))

        # Simulate cogserver environment.
        # Blending methods will be located in here.
        while 1:
            break


# Logging will be written to opencog.log in the current directory.
log.set_level('INFO')
# log.use_stdout()

inst = ShellBlending()
inst.run()