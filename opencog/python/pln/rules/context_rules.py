__author__ = 'sebastian'

from opencog.atomspace import types
from pln.rules import Rule
from pln.rules.formulas import contextFormula

"""
Rules to convert certain types of links into ContextLinks
and to convert these types of links back into ContextLinks.
"""


class InheritanceToContextRule(Rule):
    """
    InheritanceLink <TV>    EvaluationLink <TV>         SubsetLink <TV>
        ANDLink             PredicateNode A                C
            C               ListLink                       A
            A                   ANDLink
        ANDLink                     C
            C                       B
            B
    |-                      |-                          |-
    ContextLink <TV>        ContextLink <TV>            ContextLink <TV>
        C                       C                           C
        InheritanceLink         EvaluationLink              A
            A                       PredicateNode Ac
            B                       ListLink
                                        B
    """
    def __init__(self, chainer):
        a = chainer.new_variable()
        b = chainer.new_variable()
        c = chainer.new_variable()

        self.chainer = chainer
        Rule.__init__(self,
                      formula=contextFormula,
                      inputs=[chainer.link(
                          types.InheritanceLink,  # InheritanceLink
                          [chainer.link(types.AndLink, [c, a]),  # C ANDLink A
                           # C ANDLink B
                           chainer.link(types.AndLink, [c, b])])],
                      outputs=[chainer.link(
                          types.ContextLink,  # ContextLink
                          [c,
                           # InheritanceLink
                           chainer.link(types.InheritanceLink,
                                        [a, b])])])


class EvaluationToContextRule(Rule):

    def __init__(self, chainer):
        a = chainer.new_variable()
        b = chainer.new_variable()
        c = chainer.new_variable()

        self.chainer = chainer
        Rule.__init__(self,
                      formula=None,
                      inputs=[chainer.link(
                          types.EvaluationLink,   # EvaluationLink
                          [a,  # PredicateNode A
                           chainer.link(
                               types.ListLink,   # ListLink
                               # C ANDLink B
                               [chainer.link(types.AndLink, [c, b])])])],
                      outputs=[])

    # outputs is not used
    def custom_compute(self, inputs, outputs):
        [inputs] = inputs
        tv = inputs.tv

        if inputs.out[0].type == types.PredicateNode:
            predicate_a = inputs.out[0]
            concept_c = inputs.out[1].out[0].out[0]
            concept_b = inputs.out[1].out[0].out[1]

            list_link = self.chainer.link(types.ListLink,
                                          [concept_b])
            evaluation_link = self.chainer.link(types.EvaluationLink,
                                                [predicate_a, list_link])
            context_link = [self.chainer.link(types.ContextLink,
                                              [concept_c, evaluation_link])]
        return context_link, [tv]


class SubsetToContextRule(Rule):

    def __init__(self, chainer):
        a = chainer.new_variable()
        c = chainer.new_variable()

        self.chainer = chainer
        Rule.__init__(self,
                      formula=contextFormula,
                      inputs=[chainer.link(types.SubsetLink, [c, a])],
                      outputs=[chainer.link(types.ContextLink, [c, a])])


class ContextToInheritanceRule(Rule):

    def __init__(self, chainer):
        a = chainer.new_variable()
        b = chainer.new_variable()
        c = chainer.new_variable()

        self.chainer = chainer
        Rule.__init__(self,
                      formula=contextFormula,
                      inputs=[chainer.link(
                          types.ContextLink,   # ContextLink
                          [c,
                           # InheritanceLink
                           chainer.link(types.InheritanceLink,
                                        [a, b])])],
                      outputs=[chainer.link(
                          types.InheritanceLink,   # InheritanceLink
                          # C ANDLink A
                          [chainer.link(types.AndLink, [c, a]),
                           # C ANDLink B
                           chainer.link(types.AndLink, [c, b])])])


class ContextToEvaluationRule(Rule):

    def __init__(self, chainer):
        a = chainer.new_variable()
        b = chainer.new_variable()
        c = chainer.new_variable()

        self.chainer = chainer
        Rule.__init__(self,
                      formula=None,
                      inputs=[chainer.link(
                          types.ContextLink,  # ContextLink
                          [c,  # ConceptNode C
                           chainer.link(types.EvaluationLink,  # EvaluationLink
                                        [a,  # PredicateNode Ac
                                         chainer.link(types.ListLink,
                                                      [b])])])],
                      outputs=[])

    # outputs is not used
    def custom_compute(self, inputs, outputs):
        [inputs] = inputs
        tv = inputs.tv

        if inputs.out[1].out[0].type == types.PredicateNode:
            predicate_a = inputs.out[1].out[0]
            concept_c = inputs.out[0]
            concept_b = inputs.out[1].out[1].out[0]

            and_link = self.chainer.link(types.AndLink, [concept_c, concept_b])
            list_link = self.chainer.link(types.ListLink, [and_link])
            evaluation_link = [self.chainer.link(types.EvaluationLink,
                                                [predicate_a, list_link])]
        return evaluation_link, [tv]


class ContextToSubsetRule(Rule):

    def __init__(self, chainer):
        a = chainer.new_variable()
        c = chainer.new_variable()
        self.chainer = chainer
        Rule.__init__(self,
                      formula=contextFormula,
                      inputs=[chainer.link(types.ContextLink, [c, a])],
                      outputs=[])

    def custom_compute(self, inputs, outputs):
        [inputs] = inputs
        tv = inputs.tv
        if inputs.out[0].type == inputs.out[1].type == types.ConceptNode:
            concept_c = inputs.out[0]
            concept_a = inputs.out[1]
            subset_link = [self.chainer.link(types.SubsetLink,
                                             [concept_c, concept_a])]
            return subset_link, [tv]
        # if they are not ConceptNodes, just return the inputs
        return [inputs], [tv]


class ContextFreeToSensitiveRule(Rule):

    def __init__(self, chainer):
        a = chainer.new_variable()
        c = chainer.new_variable()

        self.chainer = chainer
        Rule.__init__(self,
                      formula=contextFormula,
                      inputs=[chainer.link(types.AndLink, [c, a])],
                      outputs=[chainer.link(types.ContextLink, [c, a])])