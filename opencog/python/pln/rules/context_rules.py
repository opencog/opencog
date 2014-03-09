from opencog.atomspace import types, TruthValue

import pln.rules.formulas as formulas
from pln.rule import Rule

# Todo:
# It may be better to use SubsetLinks instead of ContextLinks, or at
# least implicitly convert them.

# (Context C x).tv = (Subset C x).tv
# (Context C: Subset x y).tv = (Subset (x AND C) (y AND C))

# DeductionRule produces
# Context C: Subset x z
# using
# Context C: Subset x y
# Context C: Subset y z
# Context C: y
# Context C: z

# Special case for direct evaluation Rules.
# Subset A B requires
# Member x A, Member x B
#
# Context C: Subset A B requires
# Member x A
# Member x B
# Member x C
# or something. and then change the math.

# Or

class ContextualRule(Rule):
    def __init__(self, chainer, rule):
        self._chainer = chainer
        self.name = 'Contextual' + rule.name
        self.full_name = 'Contextual' + rule.full_name
        self._outputs = rule._outputs
        self._inputs = rule._inputs

        self.formula = rule.formula

        context = chainer.new_variable()
        self._outputs = [self.contextlink(context, out)
                         for out in self._outputs]

        is_evaluation_rule = 'EvaluationRule' in rule.name
        if is_evaluation_rule:
            raise "Direct evaluation in a context is not handled yet"
        else:
            self._inputs = [self.contextlink(context, input)
                            for input in self._inputs]

        print self.name
        print self._outputs
        print self._inputs

    def andlink(self, context, expression):
        return self._chainer.link(types.AndLink, [context, expression])

    def contextlink(self, context, expression):
        return self._chainer.link(types.ContextLink, [context, expression])

    def extract_context(self, contextlink):
        # Todo: The variable 'context' is never used. Is it supposed to
        # be returned instead of 'contextlink'?
        context = contextlink.out[0]
        expression = contextlink.out[1]

        return contextlink, expression


class AndToContextRule(Rule):
    """
    (Context C: Subset x y).tv = (Subset (x AND C) (y AND C))
    """
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()
        C = chainer.new_variable()

        link = chainer.link(link_type, [A, B])
        contextlink = chainer.link(CONTEXT_LINK, [C, link])

        andAC = chainer.link(types.AndLink, [A, C])
        andBC = chainer.link(types.AndLink, [B, C])
        input = chainer.link(link_type, [andAC, andBC])

        Rule.__init__(self,
                      formula=formulas.identityFormula,
                      outputs=[contextlink],
                      inputs=[input])

