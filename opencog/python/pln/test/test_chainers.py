from unittest import TestCase

from opencog.atomspace import AtomSpace, types, Atom
from pln.chainers import Chainer, get_attentional_focus
import utility.tree as tree

class BackwardChainerTest(TestCase):

    def setUp(self):
        self.atomspace = AtomSpace()
        self.chainer = Chainer(self.atomspace)

    def tearDown(self):
        del self.atomspace
        del self.chainer        

    def _simple_atoms_1(self):
        atoms = []
        atoms.append( self.atomspace.add_node(types.ConceptNode, "animal") )
        atoms.append( self.atomspace.add_node(types.ConceptNode, "breathe") )
        atoms.append( self.atomspace.add_link(types.InheritanceLink, [atoms[0], atoms[1]]) )

        return atoms 

    def test__selectOne(self):
        atoms = self._simple_atoms_1()
        atoms[0].av = {'sti': 1}

        atom = self.chainer._selectOne(atoms, self.atomspace)
        assert atom != None
        assert atom == atoms[0]

        atoms[1].av = {'sti': 1}
        atoms[2].av = {'sti': 1}

        atom = self.chainer._selectOne(atoms, self.atomspace)
        assert atom in atoms

    def test_find(self):
        atoms = self._simple_atoms_1()

        template = tree.tree_from_atom(atoms[2])
        assert type(template) == tree.Tree

        result = tree.find(template, self.atomspace.get_atoms_by_type(types.Atom))
        assert type(result) == list
        assert type(result[0] == Atom)

    def test_get_attentional_focus(self):
        atoms = self._simple_atoms_1()
        atoms[2].av = {'sti': 1}

        print atoms[2]

        contents = get_attentional_focus(self.atomspace)
        print contents
        assert len(contents) == 1

    def test__select_one_matching(self):
        atoms = self._simple_atoms_1()
        atoms[2].av = {'sti': 1}

        template = tree.tree_from_atom(atoms[2])
        assert type(template) == tree.Tree

        result = self.chainer._select_one_matching(template)
        assert result != None
        print result
        print type(result)
        assert type(result) == Atom
        assert result == atoms[2]

        

        atoms[0].av = {'sti': 1}

        print get_attentional_focus(self.atomspace)

        template = tree.Tree(1)
        result = self.chainer._select_one_matching(template)
        print result
        assert result != None        

