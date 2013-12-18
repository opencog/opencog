from opencog.atomspace import types, TruthValue

import pln.formulas as formulas
import pln.rules.rules as rules

import math

# Using 3rd-order probabilities, you can evaluate the other quantifiers.
#AverageLink t: F(t)
#ForAll x: F(x)

# But it's simpler to calculate the average. This could be done generally for every atom structure. You just revise together the different values for variables.

# You need to handle the variables correctly, which is annoying. (I'm not sure how to handle variables in the inputs. But any time a Rule outputs an AverageLink that already exists, it must have the SAME EXACT variable name).

# All variables MUST have an explicit scope, which makes things way easier.

class AverageCreationRule(rules.Rule):
    '''F(bar) -> AverageLink $X: F($X)'''
    def __init__(self, chainer):
        predicate_var = chainer.new_variable()
        thing_var = chainer.node(types.VariableNode, '$average_thing')

        link = chainer.link(types.EvaluationLink, [predicate_var, thing_var])
        outputs=[link]
        inputs=[chainer.link(types.AverageLink, [thing_var, link])]

        Rule.__init__(self,
            inputs=inputs,
            outputs=outputs,
            formula=formulas.identityFormula)

# Then we want to modify DeductionRule and other Rules so they support AverageLinks appropriately.

# A possible way to do it would be to make a class that modifies first-order PLN rules so that they will use blah blah.
# Possible inputs:
# InheritanceLink ConceptNode:A ConceptNode:B, A.tv (first order)
# ImplicationLink PredicateNode:A PredicateNode:B, A.tv (higher order, but trivial)

# AverageLink $x: ImplicationLink F($x) G($x)
# requires AverageLink $x: F($x)
# Or with DeductionRule:
# AverageLink $x: ImplicationLink F($x) H($x)
# requires AverageLink $x: ImplicationLink F($x) G($x)
# requires AverageLink $x: ImplicationLink G($x) H($x)
# requires AverageLink $x: F($x)
# (etc for G and H)
# Note that the variable used at the top scope must be used everywhere.


# You also need to be able to calculate AndLinks across expressions with variables in them. Same idea

# Basically, assume that an expression is quantified with an AverageLink and those variables must appear below (but what if they don't?!)

# e.g.
# AverageLink $x: AndLink ($x is a Norse god) (Norse gods exist)
# If a sub-expression doesn't contain the variable, you should just use its TruthValue.
# If it does contain the variable, you have to add an AverageLink into it.
# The sub-expressions may use the same or different names for a variable though. It might be easier not to use AverageLinks explicitly (although I guess you always have to check 


# The TruthValue for AverageLink $x: EvaluationLink F($x)
# may be different to the TV for the SatisfyingSet of F($x) because it is the average fuzzy TruthValue

# There should probably be a special class that modifies existing rules to use variables. (It doesn't depend on which rule you use, i think)


# The variable can also be in different places, and you have to check that it's in the expression, and blah blah.
# It seems annoying to do this right
# I probably need a custom_compute (and maybe more)?
# Forward chaining may be slightly easier

class HigherOrderRule(rules.Rule):
    def __init__(self, chainer, rule):
        self._chainer =  chainer
        self.name = 'HigherOrder' + rule.name
        self.full_name = 'HigherOrder' + rule.full_name
        self._outputs = rule._outputs
        self._inputs = rule._inputs

        self.formula = rule.formula

        
        self._variable = chainer.new_variable(prefix = '$average_')
        self._outputs = [self.average(out) for out in self._outputs]
        self._inputs = [self.average(input) for input in self._inputs]

        print self.name
        print self._outputs
        print self._inputs

    def average(self, expression):
        return self._chainer.link(types.AverageLink, [self._variable, expression])

    def extract_average(self, averagelink):
        variable = averagelink.out[0]
        expression = averagelink.out[1]

        return (variable, expression)

# change InheritanceLink to ImplicationLink
class DeductionRule(rules.Rule):
    '''A->B, B->C entails A->C'''
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()
        C = chainer.new_variable()

        Rule.__init__(self,
            formula= formulas.deductionIndependenceBasedFormula,
            outputs= [chainer.link(link_type, [A, C])],
            inputs=  [chainer.link(link_type, [A, B]),
                      chainer.link(link_type, [B, C]),
                      B, C])

class VariableInstantiationRule(rules.Rule):
    '''Given AverageLink $X EvaluationLink F $X
    Produce F(A) for any specific A.
    The other inputs are A and SimilarityLink $X A'''
    pass

