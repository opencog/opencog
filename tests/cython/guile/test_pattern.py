from unittest import TestCase

from opencog.atomspace import AtomSpace, TruthValue, Atom, Handle
from opencog.atomspace import types, is_a, get_type, get_type_name
from opencog.scheme_wrapper import load_scm, scheme_eval, init


class SchemeTest(TestCase):

    def setUp(self):
        self.space = AtomSpace()
        init()

    def tearDown(self):
        del self.space

    # Load several different scheme files, containin atom type
    # declarations, and utuilites. They should load just fine.
    # These don't actually put any tomes into the atomspace.
    def test_a_load_core_types(self):

        status = load_scm(self.space, "./opencog/atomspace/core_types.scm")
        self.assertTrue(status)

        status = load_scm(self.space, "./opencog/nlp/types/nlp_types.scm")
        self.assertTrue(status)

        status = load_scm(self.space, "../opencog/scm/utilities.scm")
        self.assertTrue(status)

    # Load a file that results in atoms placed in the atomspace.
    # Make sure the loaded atom is what we think it is.
    def test_b_load_file(self):

        status = load_scm(self.space, "../tests/cython/guile/basic_unify.scm")
        self.assertTrue(status)

        a1 = self.space.add_node(types.ConceptNode, "hello")
        self.assertTrue(a1)

        # Make sure the truth value is what's in the SCM file.
        expected = TruthValue(0.5, 800)
        self.assertEquals(a1.tv, expected)
        # print a1.tv, expected

    # Run some basic evaluation tests
    def test_c_eval(self):
        basic = scheme_eval_h(self.space,
            "(ConceptNode \"whatever\" (stv 0.5 0.5))")
        print "duude", basic

        a1 = self.space.add_node(types.ConceptNode, "whatever")
        self.assertTrue(a1)

        print "duude", basic == a1

        # Make sure the truth value is what's in the SCM file.
        expected = TruthValue(0.5, 800)
        self.assertEquals(a1.tv, expected)


    # Run the pattern-matcher/unifier/query-engine.
    def test_unifier(self):
        h = scheme_eval(self.space, "(cog-bind cap-deduce)")

        print h
