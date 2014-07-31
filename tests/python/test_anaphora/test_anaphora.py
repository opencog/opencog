__author__ = 'Hujie'

import unittest

from opencog.atomspace import AtomSpace, TruthValue, Atom, Handle
from opencog.atomspace import types, is_a, get_type, get_type_name
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h, __init__
from anaphora.agents.hobbs import HobbsAgent
from unittest import TestCase


__VERBOSE__ = False

# Set to True to search for needed .scm files in default IN-SOURCE build location, e.g. to write unit tests in the IDE
# Set to False to search for needed .scm files based on environment variables PROJECT_SOURCE_DIR and PROJECT_BINARY_DIR
__DEV_MODE__ = False

class AnaphoraUnitTester(TestCase):
    def setUp(self):
        self.atomspace= AtomSpace()
        self.hobbsAgent=HobbsAgent(self.atomspace)

    def tearDown(self):
        del self.atomspace
        del self.hobbsAgent

    def test_bfs(self):

        '''
        Testing the bfs function
        '''


        load_scm(self.atomspace, "tests/python/test_anaphora/data/bfs.scm")
        self.assertTrue(self.compare(['a','b','c','d','e','f','g'],self.hobbsAgent.bfs(self.atomspace.get_atoms_by_name(self,types.WordInstanceNode,'a'))))
        self.atomspace.clear()

    def test_getWords(self):

        '''
        Testing the getWords function
        '''

        load_scm(self.atomspace, "tests/python/test_anaphora/data/getWords.scm")
        self.assertTrue(self.compare(['a','b','c','d','e','f','g','h','j'],self.hobbsAgent.getWords()))
        self.atomspace.clear()

    def test_getTargets(self):

        '''
        Testing the getTargets function
        '''

        load_scm(self.atomspace, "tests/python/test_anaphora/data/getTargets.scm")
        self.assertTrue(self.compare(['a','c','d'],self.hobbsAgent.getTargets(self.hobbsAgent.getWords())))
        self.atomspace.clear()

    def test_propose(self):

        '''
        Testing the propose function
        '''

        load_scm(self.atomspace, "tests/python/test_anaphora/data/propose.scm")
        self.assertTrue(self.compare(['a','c','d'],self.hobbsAgent.getTargets(self.hobbsAgent.getWords())))
        self.atomspace.clear()
