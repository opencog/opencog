from unittest import TestCase

from opencog.atomspace import AtomSpace, TruthValue, Atom, Handle
from opencog.atomspace import types, is_a, get_type, get_type_name
from opencog.scheme_wrapper import load_scm, scheme_eval


class AtomSpaceTest(TestCase):

    def setUp(self):
        self.space = AtomSpace()

    def tearDown(self):
        del self.space

    def test_load_core_types(self):

        status = load_scm(self.space, "./opencog/atomspace/core_types.scm")
        self.assertTrue(status)

        status = load_scm(self.space, "../opencog/scm/utilities.scm")
        self.assertTrue(status)

    def test_load_file(self):

        status = load_scm(self.space, "../tests/cython/guile/basic_unify.scm")
        self.assertTrue(status)

        a1 = self.space.add_node(types.ConceptNode, "hello")
        self.assertTrue(a1)

        # Make sure the truth value is what's in the SCM file.
        expected = TruthValue(0.5, 800)
        self.assertEquals(a1.tv, expected)
        # print a1.tv, expected


