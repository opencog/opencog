import unittest
raise unittest.SkipTest("Unit test temporarily disabled - see: https://github.com/opencog/opencog/issues/442")

from unittest import TestCase

from pprint import pprint

from opencog.atomspace import AtomSpace, types, Atom
from pln.chainers import Chainer, get_attentional_focus


class ForwardChainerTest(TestCase):

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

        atom = self.chainer._selectOne(atoms)
        self.assertNotEquals(atom, None)
        self.assertEqual(atom, atoms[0])

        atom = self.chainer._selectOne(atoms)
        self.assertTrue(atom in atoms)

    def test_get_attentional_focus(self):
        atoms = self._simple_atoms_1()

        print(atoms[2])

        contents = get_attentional_focus(self.atomspace)
        print(contents)
        self.assertEqual( len(contents), 1 )

    def test__select_one_matching(self):
        atoms = self._simple_atoms_1()

        template = atoms[2]

        result = self.chainer._select_one_matching(template)
        self.assertEqual( result, atoms[2] )

        print(get_attentional_focus(self.atomspace))

        template = self.atomspace.add_node(types.VariableNode, "$v1")
        result = self.chainer._select_one_matching(template)
        self.assertNotEqual( result, None )

    def test__find_inputs_recursive(self):
        def apply(generic_inputs, generic_outputs):
            inputs = []
            outputs = []
            empty_substitution = {}
            status = self.chainer._find_inputs_recursive(inputs, outputs, generic_inputs, generic_outputs, empty_substitution)

            return (inputs, outputs)

        atoms = self._simple_atoms_1()
        
        # test it on lots of simple made up rules that include the edge cases
        # [] => []
        generic_inputs = []
        generic_outputs = []
        (inputs, outputs) = apply(generic_inputs, generic_outputs)
        self.assertEquals( inputs, [] )
        self.assertEquals( outputs, [] )

        # [animal] => []

        generic_inputs = [atoms[0]]
        generic_outputs = []
        (inputs, outputs) = apply(generic_inputs, generic_outputs)
        self.assertEquals( inputs, [atoms[0]] )
        self.assertEquals( outputs, [] )

        v1 = self.atomspace.add_node(types.VariableNode, "$v1")
        # [v1] => []
        generic_inputs = [v1]
        generic_outputs = []
        (inputs, outputs) = apply(generic_inputs, generic_outputs)
        self.assertEquals( len(inputs), 1 )
        self.assertEquals( outputs, [] )

        #from nose.tools import set_trace; set_trace()
        # [v1] => [v1]
        generic_inputs = [v1]
        generic_outputs = [v1]
        (inputs, outputs) = apply(generic_inputs, generic_outputs)
        print(str(inputs[0]))
        print(str(outputs[0]))
        self.assertEquals( len(inputs), 1 )
        self.assertEquals( len(outputs), 1 )

