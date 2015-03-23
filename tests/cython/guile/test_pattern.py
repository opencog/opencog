from unittest import TestCase

from opencog.atomspace import AtomSpace, TruthValue, Atom, Handle
from opencog.atomspace import types, is_a, get_type, get_type_name
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h, __init__


# We are poking atoms into this from the scm files, so we want
# them to still be there, later.
shared_space = AtomSpace()
__init__(shared_space)

class SchemeTest(TestCase):

    def setUp(self):
        global shared_space
        self.space = shared_space

    def tearDown(self):
        pass

    # Load several different scheme files, containing atom type
    # declarations, and utilities. They should load just fine.
    # These don't actually put any atoms into the atomspace.
    def test_a_load_core_types(self):

        # These relative paths are horridly ugly.
        # There must be a better way ...
        status = load_scm(self.space, "opencog/atomspace/core_types.scm")
        self.assertTrue(status)

        status = load_scm(self.space, "opencog/nlp/types/nlp_types.scm")
        self.assertTrue(status)

        status = load_scm(self.space, "opencog/scm/utilities.scm")
        self.assertTrue(status)

    # Load a file that results in atoms placed in the atomspace.
    # Make sure the loaded atom is what we think it is.
    def test_b_load_file(self):

        status = load_scm(self.space, "tests/cython/guile/basic_unify.scm")
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

        a1 = self.space.add_node(types.ConceptNode, "whatever")
        self.assertTrue(a1)

        # Make sure the truth value is what's in the SCM file.
        expected = TruthValue(0.5, 800)
        self.assertEquals(a1.tv, expected)

        # Actually, the atoms overall should compare.
        # print "eq", Atom(basic, self.space) == a1
        self.assertEquals(a1, Atom(basic, self.space))

        # Do it again, from a define in the scm file.
        again = scheme_eval_h(self.space, "wobbly")
        a2 = self.space.add_node(types.ConceptNode, "wobbly")
        self.assertTrue(a2)
        self.assertEquals(a2, Atom(again, self.space))


    # Run the pattern-matcher/unifier/query-engine.
    def test_unifier(self):
        h = scheme_eval_h(self.space, "cap-deduce")
        self.assertTrue(h)
        print "\nThe question is:"
        print h
        question = Atom(h, self.space)
        print question

        h = scheme_eval_h(self.space, "(cog-bind cap-deduce)")
        self.assertTrue(h)
        print h
        answer = Atom(h, self.space)
        print "\nThe answer is:"
        print answer
        self.assertEqual(answer.type, types.ListLink)
        self.assertEqual(answer.arity, 2)
