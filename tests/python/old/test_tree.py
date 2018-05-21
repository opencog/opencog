from unittest import TestCase

try:
    from opencog.atomspace import AtomSpace, TruthValue, Atom #, Handle
    from opencog.atomspace import types, get_type, get_type_name # is_a
except ImportError:
    from atomspace_remote import AtomSpace, TruthValue, Atom #, Handle
    from atomspace_remote import types, get_type, get_type_name # is_a
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
        print 'l1', self.l1

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
        var_tree = tree.Var(1)
        self.assertEquals(var_tree.is_variable(),True)
        node_tree = tree.T(self.x1)
        self.assertEquals(node_tree.is_variable(),False)

    def test_unify(self):
        T = tree.T
        V = tree.Var
        x1_template = T(self.x1)
        x1_tree = tree.tree_from_atom(self.x1)
        s = tree.unify(x1_template, x1_tree, {})
        self.assertEquals(s, {})

        x2_template = T(self.x2)
        s = tree.unify(x2_template, x1_tree, {})
        self.assertEquals(s, None)
        
        all_template = V(1)
        l2_tree = tree.tree_from_atom(self.l2)
        s = tree.unify(all_template, l2_tree, {})
        s_correct = {all_template : l2_tree}
        self.assertEquals(s, s_correct)

        t1 = V(1)
        t2 = V(2)
        s = tree.unify(t1, t2, {})
        self.assertEquals(s, {V(1):V(2)})
        
        t1 = V(1)
        t2 = V(2)
        s_correct = {V(1):V(2)}
        s = tree.unify(t1, t2, s_correct)
        self.assertEquals(s, s_correct)
        
        t1 = T('blah',V(1))
        t2 = T('blah',V(2))
        s = tree.unify(t1, t2, {})
        self.assertEquals(s, {V(1):V(2)})
        
        t1 = T('blah',V(1), V(2))
        t2 = T('blah',V(3), V(4))
        s = tree.unify(t1, t2, {})
        self.assertEquals(s, {V(1):V(3), V(2):V(4)})
        
        t1 = T('blah',V(1), V(1))
        t2 = T('blah',V(2), V(2))
        s = tree.unify(t1, t2, {})
        self.assertEquals(s, {V(1):V(2)})

    def  test_find_conj(self):
        conj = (tree.tree_from_atom(self.l1), tree.tree_from_atom(self.l2))
        
        matches = tree.find_conj(conj, self.a.get_atoms_by_type(t.Atom))
        
        self.assertEquals(len(matches), 1)
        
        if len(matches) == 1:
            first = matches[0]
            
            self.assertEquals(first.subst, {})
            self.assertEquals(first.atoms, [self.l1, self.l2])
    
    # Test whether find_conj can be used to find atoms for Psi Rules. That is not actually done in the code, but could be useful as an alternative approach.
    # (This may be obsolete; an even better approach would be to use find_matching_conjunctions)
    def  test_find_conj2(self):
        a = self.a
        
        conj = (
            a.add(t.AtTimeLink, out=[a.add(t.EvaluationLink, out=[a.add(t.PredicateNode, 'increased'), a.add(t.ListLink, out=[a.add(t.EvaluationLink, out=[a.add(t.PredicateNode, 'EnergyDemandGoal'), a.add(t.ListLink, out=[])])])]), a.add(t.TimeNode, '11210347010')]),
            a.add(t.AtTimeLink, out=[a.add(t.EvaluationLink, out=[a.add(t.PredicateNode, 'actionDone'), a.add(t.ListLink, out=[a.add(t.ExecutionLink, out=[a.add(t.GroundedSchemaNode, 'eat'), a.add(t.ListLink, out=[a.add(t.AccessoryNode, 'id_-54646')])])])]), a.add(t.TimeNode, '11210347000')]),
            a.add(t.SequentialAndLink, out=[a.add(t.TimeNode, '11210347000'), a.add(t.TimeNode, '11210347010')])
        )
        conj = tuple(map(tree.tree_from_atom, conj))

        res = tree.find_conj(conj,a.get_atoms_by_type(t.Atom))

    def  test_find_conj3(self):
        a = self.a
        
        t1 = tree.atom_from_tree(tree.new_var(), a)
        t2 = tree.atom_from_tree(tree.new_var(), a)
        action = tree.atom_from_tree(tree.new_var(), a)
        goal = tree.atom_from_tree(tree.new_var(), a)
        
        conj = (
            a.add(t.AtTimeLink, out=[a.add(t.EvaluationLink, out=[a.add(t.PredicateNode, 'actionDone'), action]), t1]),
            a.add(t.AtTimeLink, out=[a.add(t.EvaluationLink, out=[a.add(t.PredicateNode, 'increased'), a.add(t.ListLink, out=[a.add(t.EvaluationLink, out=[goal, a.add(t.ListLink, out=[])])])]), t2]),
            a.add(t.SequentialAndLink, out=[a.add(t.TimeNode, '11210347000'), a.add(t.TimeNode, '11210347010')])
        )
        conj = tuple(map(tree.tree_from_atom, conj))

        res = tree.find_conj(conj,a.get_atoms_by_type(t.Atom))

    def test_apply_rule(self):
        atoms = [self.l1, self.l2]
        
        # This is supposed to look up all Atoms of (exactly) type 'Link', and return their first outgoing atom
        link_template = tree.T('Link', 1, 2)
        first = tree.Var(1)
        result_trees = tree.apply_rule(link_template, first, atoms)
        result_correct = map(tree.tree_from_atom, [self.x1, self.l1])
        self.assertEquals(result_trees, result_correct)

    def test_standardize_apart(self):
        var1, var2 = tree.Var(1), tree.Var(2)
        tr1 = tree.T('ListLink', var1, var2)
        
        tr2 = tree.standardize_apart(tr1)
        
        print tr1
        print tr2
        
        self.assertNotEquals(tree.unify(tr1, tr2, {}),  None)
        
        var1_new, var2_new = tr2.args
        
        self.assertNotEquals(var1_new, var2_new)        
        assert var1_new not in [var1, var2]
        assert var2_new not in [var1, var2]

    def test_canonical_trees(self):
        conj = (
            tree.T('ListLink', 1, 2), 
            tree.T('ListLink', 2, 3)
        )
        
        canon = tree.canonical_trees(conj)
        print canon
