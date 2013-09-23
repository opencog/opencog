from unittest import TestCase

from opencog.atomspace import AtomSpace, types, Atom, TruthValue
import pln.rules.rules as rules
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
        rule = rules.InversionRule(self.chainer)

        (input, output) = rule.standardize_apart_input_output(self.chainer)

    def test_InversionRule(self):
        rule = rules.InversionRule(self.chainer)
        
        self._inh_animal_breathe()

        result = self.chainer._apply_forward(rule)
        print result

