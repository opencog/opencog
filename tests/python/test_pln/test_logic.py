import unittest
raise unittest.SkipTest("Unit test temporarily disabled - see: https://github.com/opencog/opencog/issues/442")

from unittest import TestCase

from opencog.atomspace import AtomSpace, types, Atom
from pln.logic import Logic

class LogicTest(TestCase):
    def setUp(self):
        self.atomspace = AtomSpace()
        self.logic = Logic(self.atomspace)

    def tearDown(self):
        del self.atomspace
        del self.logic

    def _simple_atoms(self):
        '''InheritanceLink animal breathe'''
        self.animal = self.atomspace.add_node(types.ConceptNode, "animal")
        self.breathe = self.atomspace.add_node(types.ConceptNode, "breathe")
        self.inh_animal_breathe = self.atomspace.add_link(types.InheritanceLink, [self.animal, self.breathe])

        atoms = []
        atoms.append( self.animal )
        atoms.append( self.breathe )
        atoms.append( self.inh_animal_breathe )

        return atoms

    def _what_breathes(self):
        '''InheritanceLink $v1 breathe'''
        self.v1 = self.atomspace.add_node(types.VariableNode, "$v1")
        template = self.atomspace.add_link(types.InheritanceLink, 
                        [self.v1,
                         self.breathe])
        return template

    def test_variable_stuff(self):
        var1 = self.logic.new_variable()
        self.assertTrue( var1.is_a(types.VariableNode) )
        var2 = self.logic.new_variable()
        self.assertNotEqual( var1, var2 )

        self.assertTrue(self.logic.is_variable(var1))

    def test_find(self):
        atoms = self._simple_atoms()

        template = self.inh_animal_breathe

        result = self.logic.find(template, self.atomspace.get_atoms_by_type(types.Atom))
        self.assertEquals( type(result), list )
        self.assertEquals( result, [self.inh_animal_breathe] )

        
        template = self._what_breathes()

        result = self.logic.find(template, self.atomspace.get_atoms_by_type(types.Atom))
        # it should give you the inheritancelink and the template link itself!
        self.assertEquals( result, [self.inh_animal_breathe, template] )

    def test_substitute(self):
        atoms = self._simple_atoms()

        template = self._what_breathes()

        cat = self.atomspace.add_node(types.ConceptNode, "cat")
        substitution = {self.v1: cat}

        # substitute $v1->cat in (InheritanceLink $v1 breathe)
        # producing (InheritanceLink cat breathe)
        intended_result = self.atomspace.add_link(types.InheritanceLink, [cat, self.breathe])

        result = self.logic.substitute(substitution, template)
        self.assertEqual( result, intended_result)

