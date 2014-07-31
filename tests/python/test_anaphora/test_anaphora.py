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

    def init_links_and_nodes(self):
        self.PleonasticItNode=self.atomspace.add_node(types.AnchorNode, 'Pleonastic-it', TruthValue(1.0, 100))
        self.currentPronounNode = self.atomspace.add_node(types.AnchorNode, 'CurrentPronoun', TruthValue(1.0, 100))
        self.currentTarget = self.atomspace.add_node(types.AnchorNode, 'CurrentTarget', TruthValue(1.0, 100))
        self.currentResult = self.atomspace.add_node(types.AnchorNode, 'CurrentResult', TruthValue(1.0, 100))
        self.currentProposal = self.atomspace.add_node(types.AnchorNode, 'CurrentProposal', TruthValue(1.0, 100))
        self.unresolvedReferences=self.atomspace.add_node(types.AnchorNode, 'Recent Unresolved references', TruthValue(1.0, 100))
        self.resolvedReferences=self.atomspace.add_node(types.AnchorNode, 'Resolved references', TruthValue(1.0, 100))
        self.currentResolutionNode=self.atomspace.add_node(types.AnchorNode, 'CurrentResolution', TruthValue(1.0, 100))
        self.currentResolutionLink_proposal=self.atomspace.add_link(types.ListLink, [self.currentResolutionNode, self.currentProposal], TruthValue(1.0, 100))
        self.currentResolutionLink_pronoun=self.atomspace.add_link(types.ListLink, [self.currentResolutionNode, self.currentPronounNode], TruthValue(1.0, 100))

    def tearDown(self):
        del self.atomspace
        del self.hobbsAgent

    def getWord(self,name):
        return self.atomspace.get_atoms_by_name(self,types.WordInstanceNode,name)

    def test_bfs(self):

        '''
        Testing the bfs function
        '''


        load_scm(self.atomspace, "tests/python/test_anaphora/data/bfs.scm")
        self.assertTrue(self.compare(['a','b','c','d','e','f','g'],self.hobbsAgent.bfs(self.getWord('a'))))
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

        def filter_1():

            load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#1/data_#1.scm")
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

            load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#1/data_#2.scm")
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

            load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#1/data_#3.scm")
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

            load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#1/data_#4.scm")
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

            load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#1/data_#5.scm")
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

            load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#1/data_#6.scm")
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

        def filter_2():

            load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#2/data_#1.scm")
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

            load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#2/data_#2.scm")
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

        def filter_3():

            load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#3/data_#1.scm")
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

            load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#3/data_#2.scm")
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

            load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#3/data_#3.scm")
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

        def filter_4():

            load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#4/data_#1.scm")
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

            load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#4/data_#2.scm")
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

            load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#4/data_#3.scm")
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

            load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#4/data_#4.scm")
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()


        filter_1()
        filter_2()
        filter_3()
        filter_4()


