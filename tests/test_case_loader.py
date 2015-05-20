import blending_util

__author__ = 'DongMin Kim'

from opencog.type_constructors import *
from tests.paul_sally.paul_sally import PaulSallyExample

# Make several test cases for debug.
class TestCaseMaker:

    def __init__(self, atomspace):
        self.a = atomspace
        self._make_default_concept()
        self.paul_sally_example = PaulSallyExample(self.a)

    def _make_default_concept(self):
        # Dummy truth values.
        self.atom_tv = TruthValue(1, 1)
        self.link_tv = TruthValue(1, 1)

        # - Base Concepts
        # Make space & frame concept.
        self.a_space = ConceptNode("Space", self.atom_tv)
        self.a_frame = ConceptNode("Frame", self.atom_tv)

    def make_all(self):
        self.paul_sally_example.make()

