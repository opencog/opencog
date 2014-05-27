from opencog.atomspace import types, TruthValue
import formulas
from pln.rule import Rule

# Based on Chapter 10: Higher Order Extensional Inference
# That chapter handles some cases of PLN higher order logic that don't
# require quantifiers

# ImplicationLink PredicateNode:A PredicateNode:B
# can be handled trivially using the existing first-order PLN rules.
# This is where we handle some other special cases.

# Cases with variables can be handled by GeneralEvaluationToMemberRule
# (in other file). But it would be better to do it implicitly, so
# you don't have the separate steps of converting from FOL to HOL
# and back again.


class EvaluationImplicationRule(Rule):
    """
    Evaluation is_American Ben <fuzzy tv 1>
    Implication is_American is_idiot <strength tv 2>
    |-
    Evaluation is_idiot Ben <tv3>

    This Rule does part of what the Predicate Logic ModusPonensRule does.
    Is only heuristic, and not compatible with variables as is.
    """
    def __init__(self, chainer):
       predA = chainer.new_variable()
       predB = chainer.new_variable()
       person = chainer.new_variable()

       eval1 = chainer.link(types.EvaluationLink, [predA, person])
       impl  = chainer.link(types.ImplicationLink, [predA, predB])
       eval2 = chainer.link(types.EvaluationLink, [predB, person])

       Rule.__init__(self,
                     formula=formulas.evaluationImplicationFormula,
                     outputs=[eval2],
                     inputs=[eval1, eval2, predA, predB])

'''
Similarity substitution Rule is equivalent to combining that rule with
Abduction

Deduction:
A does B
B causes C
|-
A causes C


Abduction:

A does C
A isa B
|-
B's do C

Implication A B
Implication A C (from Eval C A)
|- Abduction Rule
Implication B C


Induction:

B causes A
B causes C
|-
A is C


Maybe you should just use Mem2Inh and Inh2Mem instead (or even convert
everything into inhlinks to see what would happen!)

Eval2Implication should be a special case but would only work for
one-argument versions of the predicates...

Eval2Member + Member2Inheritance + Rules + Inheritance2Member

Have a currying Rule!
'''
