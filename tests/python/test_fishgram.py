from unittest import TestCase
# uses Python mock, installed with "sudo easy_install mock"
# http://www.voidspace.org.uk/python/mock/
from mock import patch

from opencog.atomspace import AtomSpace, TruthValue, Atom, Handle
from opencog.atomspace import types, is_a, get_type, get_type_name

import fishgram

# run doctests
import doctest
doctest.testmod(fishgram)

# For testing python modules separate from all the other tests,
# go to bin/tests, then run "ctest -v -I 56,56" to run just the Python module
# tests. Use ctest to find out the right number if the test ordering changes
# for some reason.

def add_fishgram_data(atomspace):
    a = atomspace
    # load and parse a file, or just add some dummy nodes.
    # it shouldn't be huge amounts of data, just enough to demonstrate
    # that the code is functionally correct    


class FishgramTest(TestCase):

    def setUp(self):
        self.space = AtomSpace()
        self.fishgram = fishgram.Fishgram(self.space)
        add_fishgram_data(self.space)

    def tearDown(self):
        pass

    def test_implications(self):
        i = None #self.fishgram.implications()
        self.assertEquals(i,None)

    # to patch things from an actual module before instantiating 
    #@patch('fishgram.Fishgram.run')

    @patch('fishgram.Fishgram.closed_bfs_layers')
    def test_implications_w_patch(self,mock_bfs):
        # you can set this to various values that closed_bfs_layers might sanely 
        # return, you can try insane values to test robustness
        mock_bfs.return_value = [] 

        i = self.fishgram.implications()
        self.assertEquals(mock_bfs.called,True)

    def test_notice_changes(self):
        pass
