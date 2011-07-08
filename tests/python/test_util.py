
import util

# run doctests
import doctest
doctest.testmod(util)

from unittest import TestCase
from mock import patch

from opencog.atomspace import AtomSpace, TruthValue, Atom, Handle
from opencog.atomspace import types, is_a, get_type, get_type_name

class MiscUtilTest(TestCase):
    # we only need to test supporting functions without doctests
    # namely if_ and pp

    def setUp(self):
        self.space = AtomSpace()

    def tearDown(self):
        pass

    def test_if_(self):
        self.assertEquals(util.if_(True,"a","b"),"a")
        self.assertEquals(util.if_(False,"a","b"),"b")

    def test_pp(self):
        # TODO: Jared to look at exactly what is expected from pp

        self.assertEqual(util.pp("a"),"a")
        a_dict = {'m': 'M', 'a': 'A', 'r': 'R', 'k': 'K'}
        expected_str = "{a: A, k: K, m: M, r: R}"
        self.assertEqual(util.pp(a_dict),expected_str)

        # This behaviour is odd, pretty print doesn't return a string object
        # all the time
        a_tuple_list = ("my","tuple")
        self.assertEqual(util.pp(a_tuple_list),str(a_tuple_list))


