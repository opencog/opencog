from unittest import TestCase
# uses Python mock, installed with "sudo easy_install mock"
# http://www.voidspace.org.uk/python/mock/
from mock import patch

import os
import tempfile

from opencog.atomspace import AtomSpace, TruthValue, Atom, Handle
from opencog.atomspace import types, is_a, get_type, get_type_name

import logic
import rules
from tree import T
import opencog.util

# run doctests
import doctest
doctest.testmod(logic)

# For testing python modules separate from all the other tests,
# go to bin/tests, then run "ctest -v -I 56,56" to run just the Python module
# tests. Use ctest to find out the right number if the test ordering changes
# for some reason.

class LogicTest(TestCase):

    def setUp(self):
        self.space = AtomSpace()
        
        tempfd, self.tempfn = tempfile.mkstemp()
        # close the temp file as Logger will want to manually
        # open it
        os.close(tempfd) 
        self.log = opencog.util.create_logger(self.tempfn)


    def tearDown(self):
        pass

    def test_isomorphic(self):
        a = self.space
        t = types

        x1 = rules.Rule(T(2), 
                        [T('ImplicationLink', T('EvaluationLink', a.add(t.PredicateNode, 'is_axiom'), T('ListLink', T(1000008))), T(2)),
                         T('EvaluationLink', a.add(t.PredicateNode, 'is_axiom'), T('ListLink', T(1000008)))
                        ], name='ModusPonens')
        x2 = rules.Rule(T(2), 
                        [T('ImplicationLink', T('EvaluationLink', a.add(t.PredicateNode, 'is_axiom'), T('ListLink', T(1000011))), T(2)),
                         T('EvaluationLink', a.add(t.PredicateNode, 'is_axiom'), T('ListLink', T(1000011)))
                        ], name='ModusPonens')

        self.assertTrue(x1.isomorphic(x2))
