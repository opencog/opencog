from opencog.atomspace import types, TruthValue

import pln.formulas as formulas
import pln.temporalFormulas

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
        self._outputs = outputs
        self._inputs = inputs

        self.formula = formula
        self.name = self.__class__.__name__
    
    def calculate(self, tvs):
        tv_tuples = [(tv.mean, tv.count) for tv in tvs]
        (mean, count) = self.formula(tv_tuples)
        return TruthValue(mean, count)

    def standardize_apart_input_output(self, chainer):
        new_inputs = []
        new_outputs = []
        dic = {}

        for template in self._inputs:
            new_template = chainer.standardize_apart(template, dic)
            new_inputs.append(new_template)

        for template in self._outputs:
            new_template = chainer.standardize_apart(template, dic)
            new_outputs.append(new_template)

        return (new_inputs, new_outputs)

class InversionRule(Rule):
    def __init__(self, chainer):
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
            outputs= [chainer.link(types.InheritanceLink, [B, A])],
            inputs=  [chainer.link(types.InheritanceLink, [A, B]),
                      A, B],
            formula= formulas.inversionFormula)

class DeductionRule(Rule):
    def __init__(self, chainer):
        A = chainer.new_variable()
        B = chainer.new_variable()
        C = chainer.new_variable()

        Rule.__init__(self,
            formula= formulas.deductionSimpleFormula,
            outputs= [chainer.link(types.InheritanceLink, [A, C])],
            inputs=  [chainer.link(types.InheritanceLink, [A, B]),
                      chainer.link(types.InheritanceLink, [B, C]),
                      A, B, C])

