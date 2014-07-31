__author__ = 'Hujie'

import unittest

from opencog.atomspace import AtomSpace, TruthValue, Atom, Handle
from opencog.atomspace import types, is_a, get_type, get_type_name
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h, __init__
from anaphora.agents.hobbs import HobbsAgent
from unittest import TestCase


__VERBOSE__ = False


class AnaphoraUnitTester(TestCase):

    def setUp(self):

        self.atomspace= AtomSpace()
        __init__(self.atomspace)
        data=["opencog/scm/config.scm",
              "opencog/atomspace/core_types.scm",
              "spacetime/spacetime_types.scm",
              "opencog/nlp/types/nlp_types.scm",
              "opencog/dynamics/attention/attention_types.scm",
              "opencog/embodiment/AtomSpaceExtensions/embodiment_types.scm",
              "opencog/learning/pln/pln_types.scm",
              "opencog/scm/apply.scm",
              "opencog/scm/file-utils.scm",
              "opencog/scm/persistence.scm",
              #"opencog/scm/repl-shell.scm",
              "opencog/scm/utilities.scm",
              "opencog/scm/av-tv.scm",
              "opencog/nlp/scm/type-definitions.scm",
              "opencog/nlp/scm/config.scm",
              "opencog/nlp/scm/file-utils.scm",
              "opencog/nlp/scm/nlp-utils.scm",
              "opencog/nlp/scm/disjunct-list.scm",
              "opencog/nlp/scm/processing-utils.scm",
              "opencog/nlp/scm/relex-to-logic.scm",
              ]


        for item in data:
            status=load_scm(self.atomspace, item)
        self.hobbsAgent=HobbsAgent()


    def tearDown(self):
        del self.atomspace
        del self.hobbsAgent

    def getWord(self,name):
        rv=self.atomspace.get_atoms_by_name(types.WordInstanceNode,name)
        return rv[0]

    def compare(self,list_1,list_2):
        if len(list_1)==len(list_2):
            for i in range(len(list_1)):
                if list_1[i]!=list_2[i].name:
                    return False
            return True
        else:
            return False

    #@unittest.skip("demonstrating skipping")
    def test_bfs(self):

        '''
        Testing the bfs function
        '''

        self.hobbsAgent.initilization(self.atomspace)
        self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/bfs.scm"))

        self.hobbsAgent.run(self.atomspace)
        self.assertTrue(self.compare(['a','b','c','d','e','f','g'],self.hobbsAgent.bfs(self.getWord('a'))))
        self.atomspace.clear()

    #@unittest.skip("demonstrating skipping")
    def test_getWords(self):

        '''
        Testing the getWords function
        '''

        self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/getWords.scm"))
        self.hobbsAgent.initilization(self.atomspace)
        self.assertTrue(self.compare(['a','b','c','d','e','f','g','h','j'],self.hobbsAgent.getWords()))
        self.atomspace.clear()

    #@unittest.skip("demonstrating skipping")
    def test_getTargets(self):

        '''
        Testing the getTargets function
        '''

        self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/getTargets.scm"))
        self.hobbsAgent.initilization(self.atomspace)
        self.assertTrue(self.compare(['a','c','d'],self.hobbsAgent.getTargets(self.hobbsAgent.getWords())))
        self.atomspace.clear()

    #unittest.skip("demonstrating skipping")
    def test_propose(self):

        '''
        Testing the propose function
        '''

        def filter_1():

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#1/data_#1.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#1/data_#2.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#1/data_#3.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#1/data_#4.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#1/data_#5.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#1/data_#6.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

        def filter_2():

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#2/data_#1.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#2/data_#2.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

        def filter_3():

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#3/data_#1.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent'),3))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#3/data_#2.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),3))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#3/data_#3.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),3))
            self.atomspace.clear()

        def filter_4():

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#4/data_#1.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent'),4))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#4/data_#2.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),4))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#4/data_#3.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),4))
            self.atomspace.clear()

        def filter_5():

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#5/data_#1.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent'),5))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#5/data_#2.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),5))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#5/data_#3.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),5))
            self.atomspace.clear()

        def filter_6():

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#6/data_#1.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent'),6))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#6/data_#2.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),6))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#6/data_#3.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),6))
            self.atomspace.clear()

        def filter_7():

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#7/data_#1.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent'),7))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#7/data_#2.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),7))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#7/data_#3.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),7))
            self.atomspace.clear()


        filter_1()
        filter_2()
        filter_3()
        filter_4()
        filter_5()
        filter_6()
        filter_7()

if __name__ == '__main__':
    unittest.main()