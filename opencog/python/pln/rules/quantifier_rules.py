from opencog.atomspace import types, TruthValue
import formulas
from pln.rule import Rule

# Using 3rd-order probabilities, you can evaluate the other quantifiers.
#AverageLink t: F(t)
#ForAll x: F(x)

# But it's simpler to calculate the average. This could be done
# generally for every atom structure. You just revise together
# the different values for variables.

# You need to handle the variables correctly, which is annoying.
# (I'm not sure how to handle variables in the inputs. But any time a
# Rule outputs an AverageLink that already exists, it must have the
# SAME EXACT variable name).

# All variables MUST have an explicit scope, which makes things way
# easier.


class AverageCreationRule(Rule):
    """
    F(bar) -> AverageLink $X: F($X)
    """
    def __init__(self, chainer):
        predicate_var = chainer.new_variable()
        thing_var = chainer.node(types.VariableNode, '$average_thing')

        link = chainer.link(types.EvaluationLink, [predicate_var, thing_var])
        outputs = [link]
        inputs = [chainer.link(types.AverageLink, [thing_var, link])]

        Rule.__init__(self,
                      inputs=inputs,
                      outputs=outputs,
                      formula=formulas.identityFormula)

# Then we want to modify DeductionRule and other Rules so they support
# AverageLinks appropriately.

# A possible way to do it would be to make a class that modifies
# first-order PLN rules so that they will use blah blah.
# Possible inputs:
# InheritanceLink ConceptNode:A ConceptNode:B, A.tv (first order)
# ImplicationLink PredicateNode:A PredicateNode:B, A.tv
# (higher order, but trivial)

# AverageLink $x: ImplicationLink F($x) G($x)
# requires AverageLink $x: F($x)
# Or with DeductionRule:
# AverageLink $x: ImplicationLink F($x) H($x)
# requires AverageLink $x: ImplicationLink F($x) G($x)
# requires AverageLink $x: ImplicationLink G($x) H($x)
# requires AverageLink $x: F($x)
# (etc for G and H)
# Note that the variable used at the top scope must be used everywhere.


# You also need to be able to calculate AndLinks across expressions
# with variables in them. Same idea

# Basically, assume that an expression is quantified with an
# AverageLink and those variables must appear below (but what if they
# don't?!)

# e.g.
# AverageLink $x: AndLink ($x is a Norse god) (Norse gods exist)
# If a sub-expression doesn't contain the variable, you should just use
# its TruthValue. If it does contain the variable, you have to add an
# AverageLink into it. The sub-expressions may use the same or
# different names for a variable though. It might be easier not to use
# AverageLinks explicitly (although I guess you always have to check


# The TruthValue for AverageLink $x: EvaluationLink F($x)
# may be different to the TV for the SatisfyingSet of F($x) because it
# is the average fuzzy TruthValue

# There should probably be a special class that modifies existing rules
# to use variables. (It doesn't depend on which rule you use, i think)


# The variable can also be in different places, and you have to check
# that it's in the expression, and blah blah. It seems annoying to do
# this right I probably need a custom_compute (and maybe more)?
# Forward chaining may be slightly easier


# you need to know the probability for $a. maybe use the default term
# probability?
class HigherOrderRule(Rule):
    def __init__(self, chainer, rule):
        self._chainer =  chainer
        self.name = 'HigherOrder' + rule.name
        self.full_name = 'HigherOrder' + rule.full_name
        self._outputs = rule._outputs
        self._inputs = rule._inputs

        self.formula = rule.formula
        
        self._variable = chainer.new_variable(prefix='$average_')
        self._outputs = [self.average(out) for out in self._outputs]
        self._inputs = [self.average(input) for input in self._inputs]

        print self.name
        print self._outputs
        print self._inputs

    def average(self, expression):
        return self._chainer.link(types.AverageLink,
                                  [self._variable, expression])

    # Todo: Should this be marked as a static method?
    def extract_average(self, averagelink):
        variable = averagelink.out[0]
        expression = averagelink.out[1]

        return variable, expression


# Todo: Why is there a 'DeductionRule' both here and in
# 'inheritance_rules.py' (formerly 'rules.py')?
# change InheritanceLink to ImplicationLink
class DeductionRule(Rule):
    """
    A->B, B->C entails A->C
    """
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()
        C = chainer.new_variable()

        Rule.__init__(self,
                      formula=formulas.deductionIndependenceBasedFormula,
                      outputs=[chainer.link(link_type, [A, C])],
                      inputs=[chainer.link(link_type, [A, B]),
                              chainer.link(link_type, [B, C]),
                              B, C])


# Todo: Has not been implemented
class VariableInstantiationRule(Rule):
    """
    Given AverageLink $X EvaluationLink F $X
    Produce F(A) for any specific A.
    The other inputs are A and SimilarityLink $X A
    """
    pass



class ScholemRule(Rule):
    """
    Exist VariableNode:$cat
    Eval PredicateNode:is_grey VariableNode:$cat
    |-
    Eval PredicateNode:is_grey ConceptNode:cat_instance_1234
    """
    def __init__(self, chainer):
        quantified_var = chainer.new_variable()
        expr = chainer.new_variable()

        self._chainer = chainer

        # output is not produced, it just makes backchaining easier
        Rule.__init__(self,
                      formula=None,
                      inputs=[chainer.link(types.ExistsLink,
                                           [quantified_var, expr])],
                      outputs=[expr])
    
    # Todo: The variable 'outputs' is not used
    def custom_compute(self, inputs, outputs):
        [exist_link] = inputs
        [var, expr] = exist_link.out
        
        prefix = 'scholem_' + var.name + '_instance_'
        node = self._chainer.atomspace.add_node(types.ConceptNode,
                                                prefix,
                                                prefixed=True)
        
        new_expr = self._chainer.substitute({var: node}, expr)

        return [new_expr, exist_link.tv]

# Another really fun thing.
# Skolemization.
'''
Exists $cat: Inherits $cat cat
|-
Inherits cat_1 cat

You can do something similar for other expressions too. It's really
cool and possibly even useful! (For making inference less annoying
by removing variables!)

AverageLink $person
    Implication
        Member $person person
        Member $person breathing

AverageLink $thing (stv 0.5 0.1234)
    Eval I_know_about $thing

Eval PredicateNode:I_know_about ConceptNode:things_i_know_about

ForAll $person
    ImplicationLink
        AndLink
            Member $person parent
            Member $person female
    Member $person mother
|-
#InheritanceLink (AndLink parent female) mother

ForAll $mother
    ImplicationLink
        Eval isMother $mother
        Eval isFemale $mother
<=>
InheritanceLink mother female
or...

ImplicationLink
    Eval isMother Mothers
    Eval isFemale Mothers

Eval isMother Mothers (stv 1 1)
|-
Eval isFemale Mothers

M2I|-
Inheritance Mothers Female



ForAll $person (stv 1 1)
    EquivalenceLink
        AndLink
            Member $mother parent
            Member $mother female
    Member $mother mother
|- (convert ForAll to Average)
AverageLink $person (stv 1 1)
    EquivalenceLink
        AndLink
            Member $mother parent
            Member $mother female
    Member $mother mother
|- (average skolemization)
EquivalenceLink
    AndLink
        Member mothers parent
        Member mothers female
    Member mothers mother


or, automatically use the LHS (or RHS) of the EquivalenceLink as a
concept:

ForAll $person (stv 1 1)
    EquivalenceLink
        Member $mother mother
        AndLink
            Member $mother parent
            Member $mother female

SimilarityLink (stv 1 1)
    mother
    AndLink
        parent
        female

# since animals are non-human persons
AverageLink (stv 0.01 1)
    EquivalenceLink
        Member $person person
        Member $person human

EquivalenceLink
    Member asdf person
    Member asdf human



AverageLink $thing <0.5>
    AndLink
        Inheritance $thing meaningless
        Inheritance $thing hopeless
        Inheritance $thing made_by_humans

(ConceptNode "1234" (stv 0.5))
[ i.e. Subset Universe 1234 <0.5>]

Inheritance ConceptNode:1234 meaningless <1.0>
Inheritance ConceptNode:1234 hopeless <1.0>


# then use deduction/inversion to find the probability of
# Subset duck 1234
# Subset robot 1234


# having concepts with an intension and extension would be MUCH more
useful than SUMO-style definitions!


#AverageLink $anything <0.5>
#    Member $anything 1234



converting it into opencog-ish concepts would make inference easier
(maybe?) (if support for variables is broken somehow)
it would mean you could do deduction and induction and calculating
similarity links more easily.

except there's not enough properties info to direct-evaluate
interesting similarity links, probably
'''
