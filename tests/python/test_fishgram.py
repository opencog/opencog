from unittest import TestCase
# uses Python mock, installed with "sudo easy_install mock"
# http://www.voidspace.org.uk/python/mock/
from mock import patch

import os
import tempfile

from opencog.atomspace import AtomSpace, TruthValue, Atom, Handle
from opencog.atomspace import types, is_a, get_type, get_type_name
import opencog.util

import fishgram, tree, adaptors

# run doctests
import doctest
doctest.testmod(fishgram)

# For testing python modules separate from all the other tests,
# go to bin/tests, then run "ctest -v -I 56,56" to run just the Python module
# tests. Use ctest to find out the right number if the test ordering changes
# for some reason.

def add_fishgram_data(atomspace):
    a = atomspace
    t = types
    # load and parse a file, or just add some dummy nodes.
    # it shouldn't be huge amounts of data, just enough to demonstrate
    # that the code is functionally correct    

class FishgramTest(TestCase):

    def setUp(self):
        self.space = AtomSpace()
        self.fishgram = fishgram.Fishgram(self.space)
        add_fishgram_data(self.space)
        
        tempfd, self.tempfn = tempfile.mkstemp()
        # close the temp file as Logger will want to manually
        # open it
        os.close(tempfd) 
        self.log = opencog.util.create_logger(self.tempfn)


    def tearDown(self):
        pass

    def test_bfs(self):
#        conj = (
#            a.add(t.AtTimeLink, out=[a.add(t.TimeNode, '11210347010'), a.add(t.EvaluationLink, out=[a.add(t.PredicateNode, 'increased'), a.add(t.ListLink, out=[a.add(t.EvaluationLink, out=[a.add(t.PredicateNode, 'EnergyDemandGoal'), a.add(t.ListLink, out=[])])])])]),
#            a.add(t.AtTimeLink, out=[a.add(t.TimeNode, '11210347000'), a.add(t.EvaluationLink, out=[a.add(t.PredicateNode, 'actionDone'), a.add(t.ListLink, out=[a.add(t.ExecutionLink, out=[a.add(t.GroundedSchemaNode, 'eat'), a.add(t.ListLink, out=[a.add(t.AccessoryNode, 'id_-54646')])])])])]),
#            a.add(t.SequentialAndLink, out=[a.add(t.TimeNode, '11210347000'), a.add(t.TimeNode, '11210347010')])
#            )
#        conj = tuple(map(tree.tree_from_atom, conj))

#res = tree.find_conj(conj,a.get_atoms_by_type(t.Atom))
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

    #def test_make_psi_rule(self):
    #    T = tree.T
    #    a = self.space.add
    #    t = types
    #    
    #    bark =  T('ExecutionLink',
    #                a(t.GroundedSchemaNode, name='bark'), 
    #                T('ListLink')
    #            )
    #    
    #    goal = T('EvaluationLink',
    #                            a(t.PredicateNode, name = 'EnergyDemandGoal')
    #            )
    #    
    #    action = T('AtTimeLink', 1,
    #                T('EvaluationLink',
    #                    a(t.PredicateNode, name='actionDone'),
    #                    T('ListLink',
    #                       bark
    #                     )
    #                )
    #            )
    #    seq_and = T('SequentialAndLink', 1, 2) # two TimeNodes
    #    result =    T('AtTimeLink',
    #                    2,
    #                    T('EvaluationLink',
    #                        a(t.PredicateNode, name='increased'),
    #                        T('ListLink',
    #                            goal
    #                        )
    #                    )
    #                )
    #    
    #    premises = (action, seq_and)
    #    conclusion = result
    #    
    #    print 'premises, conclusion:'
    #    print premises
    #    print conclusion
    #    premises2, conclusion2 = self.fishgram.make_psi_rule(premises, conclusion)
    #    print premises2, conclusion2
    #    
    #    ideal_premises = (bark, )
    #    ideal_conclusion =  goal
    #    
    #    self.assertEquals(premises2, ideal_premises)
    #    self.assertEquals(conclusion2, ideal_conclusion)
    #
    #def test_lookup_causal_patterns(self):
    #    T = tree.T
    #    a = self.space.add
    #    t = types
    #    
    #    bark =  T('ExecutionLink',
    #        a(t.GroundedSchemaNode, name='bark'), 
    #        T('ListLink')
    #    )
    #    
    #    action = T('AtTimeLink', 1,
    #                    T('EvaluationLink',
    #                        a(t.PredicateNode, name='actionDone'),
    #                        T('ListLink',
    #                           bark
    #                         )
    #                    )
    #                )
    #    seq_and = T('SequentialAndLink', 1, 2) # two TimeNodes
    #    increase = T('AtTimeLink',
    #                 2,
    #                 T('EvaluationLink',
    #                        a(t.PredicateNode, name='increased'),
    #                        T('ListLink',
    #                            T('EvaluationLink',
    #                                a(t.PredicateNode, name = 'EnergyDemandGoal')
    #                            )
    #                        )
    #                    )
    #                 )
    #    
    #    # Add the pattern (still with some variables) into the ForestExtractor results, then see if it can be looked up correclty
    #    ideal_result = (action, seq_and, increase)
    #    for x in ideal_result:
    #        # No embedding actually needs to be recorded, as long as the tree itself is there
    #        self.fishgram.forest.tree_embeddings[x] = []
    #    
    #    result = next(self.fishgram.lookup_causal_patterns())
    #    print result
    #    assert tree.isomorphic_conjunctions(result, ideal_result)
