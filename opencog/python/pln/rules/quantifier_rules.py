from opencog.atomspace import types, TruthValue

import pln.formulas as formulas
import pln.rules.rules as rules

import math

# I think that quantifiers and even variables are overkill for most situations. You can just create new conceptnodes and use pln term logic. And that works for most commonsense inferences.

class QuantifierCreationRule(Rule):
    def __init__(self, chainer, quantifier=types.AverageLink):
        predicate_var = chainer.new_variable()
        thing_var = chainer.node(types.VariableNode, '$average_thing')

        link = chainer.link(types.EvaluationLink, [predicate_var, thing_var])
        outputs=[link]
        inputs=[chainer.link(quantifier, [thing_var, link])]

        # uses the wrong formula
        Rule.__init__(self,
            inputs=inputs,
            outputs=outputs,
            formula=formulas.identityFormula)

