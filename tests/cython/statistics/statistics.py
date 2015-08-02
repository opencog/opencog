__author__ = 'DongMin Kim'

from opencog.atomspace import *
from opencog.logger import log

# Only run the unit tests if the required dependencies have been installed
# (see: https://github.com/opencog/opencog/issues/337)
try:
    __import__("nose.tools")
except ImportError:
    import unittest

    raise unittest.SkipTest(
        "ImportError exception: " +
        "Can't find Nose. " +
        "make sure the required dependencies are installed."
    )
else:
    # noinspection PyPackageRequirements
    from nose.tools import *

try:
    __import__("opencog.statistics")
except ImportError:
    import unittest
    raise unittest.SkipTest(
        "ImportError exception: " +
        "Can't find statistics python binding library. " +
        "make sure the required dependencies are installed."
    )
else:
    from opencog.statistics import PyDataProviderAtom


# Same with tests/learning/statistics/StatisticsUTest.cxxtest
class TestStatistics:
    DEBUG = False
    N_GRAMS = 3
    ORDERED = False

    a = None
    provider = None

    test_case = list()
    aaa = None
    bbb = None
    ccc = None
    ddd = None
    eee = None
    fff = None

    def setUp(self):
        if self.DEBUG:
            log.use_stdout(True)

        self.a = AtomSpace()
        self.provider = PyDataProviderAtom(self.N_GRAMS, self.ORDERED)

        self.aaa = self.a.add_node(types.ConceptNode, "aaa")
        self.bbb = self.a.add_node(types.ConceptNode, "bbb")
        self.ccc = self.a.add_node(types.ConceptNode, "ccc")
        self.ddd = self.a.add_node(types.ConceptNode, "ddd")
        self.eee = self.a.add_node(types.ConceptNode, "eee")
        self.fff = self.a.add_node(types.ConceptNode, "fff")
        self.test_case = [
            self.aaa, self.bbb, self.ccc,
            self.ddd, self.eee, self.fff
        ]

    def tearDown(self):
        if self.DEBUG:
            self.print_detail(self.provider)

        del self.provider
        del self.a

    def print_detail(self, provider):
        log.info(provider.print_data_map())

    def test_data_provider(self):
        log.info("Statistics> test_data_provider")
        # Add one data
        is_first_insert = self.provider.addOneMetaData(self.aaa)
        assert_true(is_first_insert)

        # Re-add one data
        is_first_insert = self.provider.addOneMetaData(self.aaa)
        assert_false(is_first_insert)

        for case in self.test_case:
            self.provider.addOneMetaData(case)
        assert_equal(self.provider.mDataSetSize, 6)

        # Add data's count (n_gram = 2)
        arr_2_gram_1 = [self.aaa, self.bbb]
        arr_2_gram_2 = [self.aaa, self.ccc]
        self.provider.addOneRawDataCount(arr_2_gram_1, 2)
        self.provider.addOneRawDataCount(arr_2_gram_2, 1)

        # Add data's count (n_gram = 3)
        arr_3_gram_1 = [self.aaa, self.bbb, self.ccc]
        self.provider.addOneRawDataCount(arr_3_gram_1, 1)

        # Test key vector
        key_vector_1 = self.provider.makeKeyFromData(self.test_case)
        assert_equal(len(key_vector_1), 6)

        combination_array = [True, False, True, False, True, False]
        key_vector_2 = self.provider.makeKeyFromData(
            self.test_case, combination_array)
        assert_equal(len(key_vector_2), 3)

        value_vector = self.provider.makeDataFromKey(self.a, key_vector_1)
        assert_equal(len(value_vector), 6)
