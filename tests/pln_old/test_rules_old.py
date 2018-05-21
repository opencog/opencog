import unittest
raise unittest.SkipTest("Unit test temporarily disabled - see: https://github.com/opencog/opencog/issues/442")

from unittest import TestCase

from opencog.atomspace import AtomSpace, types, Atom, TruthValue
from pln.rules import *
from pln.chainers import Chainer

class RulesTest(TestCase):
    def setUp(self):
        self.atomspace = AtomSpace()
        self.chainer = Chainer(self.atomspace)

    def tearDown(self):
        del self.atomspace
        del self.chainer

    def _inh_animal_breathe(self):
        '''InheritanceLink animal breathe'''
        default_av = {'sti':1}
        self.animal = self.atomspace.add_node(types.ConceptNode, "animal")
        self.breathe = self.atomspace.add_node(types.ConceptNode, "breathe")
        self.inh_animal_breathe = self.atomspace.add_link(types.InheritanceLink, [self.animal, self.breathe])

        self.animal.tv = TruthValue(0.1, 1)
        self.breathe.tv = TruthValue(0.1, 1)
        self.inh_animal_breathe.tv = TruthValue(1, 1)

        atoms = []
        atoms.append( self.animal )
        atoms.append( self.breathe )
        atoms.append( self.inh_animal_breathe )

        for atom in atoms:
            atom.av = default_av

        return atoms

#    def _apply_rule(self, rule, 

    def test_standardize_apart_input_output(self):
        rule = InversionRule(self.chainer, types.InheritanceLink)

        (input, output) = rule.standardize_apart_input_output(self.chainer)

    def test_InversionRule(self):
        rule = InversionRule(self.chainer, types.InheritanceLink)
        
        self._inh_animal_breathe()

        result = self.chainer._apply_forward(rule)
        print(result)

    def test_InversionRule_backward(self):
        rule = InversionRule(self.chainer, types.InheritanceLink)
        
        self._inh_animal_breathe()
        self.inh_breathe_animal = self.atomspace.add_link(types.InheritanceLink, [self.breathe, self.animal])
        self.inh_breathe_animal.av = {'sti':1}

        result = self.chainer._apply_backward(rule)
        print(result)

    def disabled_test_rules_generically(self):
        '''See what happens if you give a rule the generic inputs. This makes sure that the rule and formula don't have any basic code errors, but doesn't check that they do the right thing.'''
        def apply_rule(rule):
            generic_inputs = rule.inputs
            generic_outpus = rule.outputs

            # Take the generic required input atoms and give them boring TVs
            for atom in generic_inputs:
                atom.av = {'sti':1}
                atom.tv = TruthValue(1, 1)

            status = self.chainer._apply_forward(rule)

            self.assertNotEquals(status, None)

            return None

        for rule in self.chainer.rules:
            apply_rule(rule)
