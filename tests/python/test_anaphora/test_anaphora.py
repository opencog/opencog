__author__ = 'Hujie'

import unittest

from opencog.atomspace import AtomSpace, TruthValue, Atom, Handle
from opencog.atomspace import types, is_a, get_type, get_type_name
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h, __init__
from anaphora.agents.hobbs import HobbsAgent
#from agents.hobbs import HobbsAgent
from unittest import TestCase


__VERBOSE__ = False

# Set to True to search for needed .scm files in default IN-SOURCE build location, e.g. to write unit tests in the IDE
# Set to False to search for needed .scm files based on environment variables PROJECT_SOURCE_DIR and PROJECT_BINARY_DIR
__DEV_MODE__ = False

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

    def getWord(self,name,type=types.WordInstanceNode):
        rv=self.atomspace.get_atoms_by_name(type,name)
        return rv[0]

    def compare(self,list_1,list_2):
        if len(list_1)==len(list_2):
            for i in range(len(list_1)):
                if list_1[i]!=list_2[i].name:
                    return False
            return True
        else:
            return False

    #@unittest.skip("debugging skipping")
    def test_bfs(self):

        '''
        Testing the bfs function
        '''

        self.hobbsAgent.initilization(self.atomspace)
        self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/bfs.scm"))

        self.hobbsAgent.run(self.atomspace)
        self.assertTrue(self.compare(['a','b','c','d','e','f','g'],self.hobbsAgent.bfs(self.getWord('a'))))
        self.atomspace.clear()

    #@unittest.skip("debugging skipping")
    def test_getWords(self):

        '''
        Testing the getWords function
        '''

        self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/getWords.scm"))
        self.hobbsAgent.initilization(self.atomspace)
        self.assertTrue(self.compare(['a','b','c','d','e','f','g','h','j'],self.hobbsAgent.getWords()))
        self.atomspace.clear()

    #@unittest.skip("debugging skipping")
    def test_propose(self):

        '''
        Testing the propose function
        '''

        def filter_1():

            print("Testing filter #1...")
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

            print("Testing filter #2...")
            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#2/data_#1.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#2/data_#2.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent')))
            self.atomspace.clear()

        def filter_3():

            print("Testing filter #3...")
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

            print("Testing filter #4...")
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

            print("Testing filter #5...")
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

            print("Testing filter #6...")
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

            print("Testing filter #7...")
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

        def filter_8():

            print("Testing filter #8...")
            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#8/data_#1.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent'),8))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#8/data_#2.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),8))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#8/data_#3.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),8))
            self.atomspace.clear()

        def filter_9():

            print("Testing filter #9...")
            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#9/data_#1.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent'),9))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#9/data_#2.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),9))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#9/data_#3.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),9))
            self.atomspace.clear()

        def filter_10():

            print("Testing filter #10...")
            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#10/data_#1.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent'),10))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#10/data_#2.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),10))
            self.atomspace.clear()

        def filter_11():

            print("Testing filter #11...")
            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#11/data_#1.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent'),11))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#11/data_#2.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),11))
            self.atomspace.clear()

        def filter_12():

            print("Testing filter #12...")
            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#12/data_#1.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent',types.ParseNode),12))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#12/data_#2.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),12))
            self.atomspace.clear()

        def filter_13():

            print("Testing filter #13...")
            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#13/data_#1.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent'),13))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#13/data_#2.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),13))
            self.atomspace.clear()

        def filter_14():

            print("Testing filter #14...")
            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#14/data_#1.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent'),14))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#14/data_#2.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),14))
            self.atomspace.clear()

        def filter_15():

            print("Testing filter #15...")
            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#15/data_#1.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent'),15))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#15/data_#2.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),15))
            self.atomspace.clear()

        def filter_16():

            print("Testing filter #16...")
            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#16/data_#1.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent'),16))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#16/data_#2.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),16))
            self.atomspace.clear()

        def filter_17():

            print("Testing filter #17...")
            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#17/data_#1.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent'),17))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#17/data_#2.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),17))
            self.atomspace.clear()

        def filter_18():

            print("Testing filter #18...")
            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#18/data_#1.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertFalse(self.hobbsAgent.propose(self.getWord('antecedent'),18))
            self.atomspace.clear()

            self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/propose/filter-#18/data_#2.scm"))
            self.hobbsAgent.initilization(self.atomspace)
            self.assertTrue(self.hobbsAgent.propose(self.getWord('antecedent'),18))
            self.atomspace.clear()

        filter_1()
        filter_2()
        filter_3()
        filter_4()
        filter_5()
        filter_6()
        filter_7()
        filter_8()
        filter_9()
        filter_10()
        filter_11()
        filter_12()
        filter_13()
        filter_14()
        filter_15()
        filter_16()
        filter_17()
        filter_18()

    #@unittest.skip("debugging skipping")
    def test_pleonastic_if(self):

        self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/pleonastic_it/data_#1.scm"))
        self.hobbsAgent.initilization(self.atomspace)
        self.assertTrue(self.hobbsAgent.pleonastic_it(self.getWord('it')))
        self.atomspace.clear()

        self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/pleonastic_it/data_#2.scm"))
        self.hobbsAgent.initilization(self.atomspace)
        self.assertTrue(self.hobbsAgent.pleonastic_it(self.getWord('it')))
        self.atomspace.clear()

        self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/pleonastic_it/data_#3.scm"))
        self.hobbsAgent.initilization(self.atomspace)
        self.assertTrue(self.hobbsAgent.pleonastic_it(self.getWord('it')))
        self.atomspace.clear()

        self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/pleonastic_it/data_#4.scm"))
        self.hobbsAgent.initilization(self.atomspace)
        self.assertFalse(self.hobbsAgent.pleonastic_it(self.getWord('it')))
        self.atomspace.clear()

        self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/pleonastic_it/data_#5.scm"))
        self.hobbsAgent.initilization(self.atomspace)
        self.assertFalse(self.hobbsAgent.pleonastic_it(self.getWord('it')))
        self.atomspace.clear()

    #@unittest.skip("debugging skipping")
    def test_conjunctions(self):

        self.assertTrue(load_scm(self.atomspace, "tests/python/test_anaphora/data/conjunction.scm"))
        self.hobbsAgent.initilization(self.atomspace)
        self.assertFalse(self.hobbsAgent.pleonastic_it(self.getWord('waitresses')))
        self.atomspace.clear()


if __name__ == '__main__':
    unittest.main()