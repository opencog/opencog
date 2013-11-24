from opencog.atomspace import types, TruthValue

import pln.formulas as formulas
import pln.rules.rules as rules

import math

# I think that quantifiers and even variables are overkill for most situations. You can just create new conceptnodes and use pln term logic. And that works for most commonsense inferences.

# Using 3rd-order probabilities, you can evaluate the other quantifiers.
#AverageLink t: F(t)
#ForAll x: F(x)

# But it's simpler to calculate the average. This could be done generally for every atom structure. You just revise together the variables

class AverageCreationRule(Rule):
    def __init__(self, chainer):
        predicate_var = chainer.new_variable()
        thing_var = chainer.node(types.VariableNode, '$average_thing')

        link = chainer.link(types.EvaluationLink, [predicate_var, thing_var])
        outputs=[link]
        inputs=[chainer.link(types.AverageLink, [thing_var, link])]

        # uses the wrong formula
        Rule.__init__(self,
            inputs=inputs,
            outputs=outputs,
            formula=formulas.identityFormula)


class VariableInstantiationRule(Rule):
    '''Given AverageLink $X EvaluationLink F $X
    Produce F(A) for any specific A.
    The other inputs are A and SimilarityLink $X A'''
    pass

