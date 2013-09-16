import pln.formulas as formulas
import pln.temporalFormulas
import utility.tree
from utility.tree import new_var, T # Tree constructor

import math

class Rule(object):

    def __init__ (self, outputs, inputs, formula):
        '''@outputs is one or more Trees representing the structure of Atom (usually a Link)
    to be produced by this Rule. If it's a variable then any kind of Atom
    can be produced.
    @inputs (list of Trees) specifies what kinds of Atoms
    are necessary to produce it. There should be variables used in the head
    that also appear in the goals. This means they will be the same atom.
    It's OK to reuse the same variable numbers in different Rules - they'll
    be converted to unique variables automatically.
    You can use either @formula or @tv. You specify a formula method from formulas.py;
    it will be called with the TVs of the relevant Atoms, to calculate the TruthValue for
    the resulting Atom.'''
        self.outputs = outputs
        self.inputs = inputs

        self.name = self.__class__.__name__
    
    def standardize_apart(self):
        '''Create a new version of the Rule where both the head and goals use new
        variables. Important for unification.'''
        head_goals = (self.head,)+tuple(self.goals)
        tmp = standardize_apart(head_goals)
        new_version = Rule(tmp[0], tmp[1:], name=self.name, tv = self.tv,
                           formula=self.formula, match = self.match)

        return new_version

    def subst(self, s):
	'''Create a new Rule where substitution s has been substituted into both the head and goals.'''
        new_head = subst(s, self.head)
        new_goals = list(subst_conjunction(s, self.goals))
        new_rule = Rule(new_head, new_goals, name=self.name, tv = self.tv,
                        formula = self.formula, match = self.match)
        return new_rule

class InversionRule(Rule):
    def __init__(self):
        A = new_var()
        B = new_var()

        Rule.__init__(self,
            outputs= [T('InheritanceLink', B, A)],
            inputs=  [T('InheritanceLink', A, B)],
            formula= formulas.inversionFormula)

class DeductionRule(Rule):
    def __init__(self):
        A = new_var()
        B = new_var()
        C = new_var()

        Rule.__init__(self,
            formula= formulas.deductionFormula,
            outputs= [T('InheritanceLink', A, C)],
            inputs=  [T('InheritanceLink', A, B),
                      T('InheirtanceLink', B, C)])

