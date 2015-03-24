from unittest import TestCase

from opencog.atomspace import AtomSpace
from opencog.utilities import initialize_opencog, finalize_opencog

__author__ = 'Curtis Faith'


class UtilitiesTest(TestCase):

    def setUp(self):
        self.atomspace = AtomSpace()
 
    def tearDown(self):
        del self.atomspace

    def test_initialize_finalize(self):
        initialize_opencog(self.atomspace)
        finalize_opencog()
        