from unittest import TestCase
# uses Python mock, installed with "sudo easy_install mock"
# http://www.voidspace.org.uk/python/mock/
from mock import patch, patch_object

import opencog.atomspace
from opencog.atomspace import AtomSpace, TruthValue, Atom, Handle
from opencog.atomspace import types, is_a, get_type, get_type_name
t=types

import tree

# run any doctests
import doctest
doctest.testmod(tree)

class TreeTest(TestCase):

    def setUp(self):
        self.a = AtomSpace()
        self.x1 = self.a.add(t.ConceptNode,"test1")
        self.x2 = self.a.add(t.ConceptNode,"test2")
        self.l1 = self.a.add(t.Link, out=[self.x1,self.x2])
        self.l2 = self.a.add(t.Link, out=[self.l1,self.x2])

    def tearDown(self):
        del self.a

    def test_atom_tree(self):
        node_tree = tree.tree_from_atom(self.x1)
        self.assertEquals(node_tree.is_leaf(), True)

    def test_link_tree(self):
        l_tree = tree.tree_from_atom(self.l1)

        self.assertEquals(l_tree.is_leaf(), False)

        # should be something like ('Link', 17, 18)
        x = l_tree.to_tuple()
        self.assertEquals(len(x), 3 )

    def test_link_to_link_tree(self):
        l_tree = tree.tree_from_atom(self.l2)
        
        self.assertEquals(l_tree.is_leaf(), False)

        # should be something like ('Link', ('Link', 13, 14), 14)
        x = l_tree.to_tuple()
        self.assertEquals(len(x), 3)
        self.assertEquals(len(x[1]), 3)
        self.assertEquals(x[1][2], x[2])

    def test_compare(self):
        l_tree1 = tree.tree_from_atom(self.l1)
        l_tree = tree.tree_from_atom(self.l2)

        self.assertEquals(l_tree1 > l_tree, False)
        self.assertEquals(l_tree1 < l_tree, True)


    def test_coerce_tree(self):
        node_tree = tree.tree_from_atom(self.x1)
        print str(node_tree)
        self.assertEquals(tree.coerce_tree(node_tree),node_tree)
        self.assertEquals(tree.coerce_tree(self.x1),node_tree)
        self.assertEquals(tree.coerce_tree("tree").op,"tree")

    def test_is_variable(self):
        var_tree = tree.tree(1)
        self.assertEquals(var_tree.is_variable(),True)
        node_tree = tree.tree(self.x1)
        self.assertEquals(node_tree.is_variable(),False)


