__author__ = 'sebastian'

from opencog.atomspace import types, TruthValue, get_type_name
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
            A                       PredicateNode A
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
                      inputs=[chainer.link(types.InheritanceLink,  # InheritanceLink
                                           [chainer.link(types.AndLink, [c, a]),  # C ANDLink A
                                            chainer.link(types.AndLink, [c, b])])],  # C ANDLink B
                      outputs=[chainer.link(types.ContextLink,  # ContextLink
                                            [c,
                                             chainer.link(types.InheritanceLink,  # InheritanceLink
                                                          [a, b])])])


class EvaluationToContextRule(Rule):

    def __init__(self, chainer):
        a = chainer.new_variable()
        b = chainer.new_variable()
        c = chainer.new_variable()

        self.chainer = chainer
        Rule.__init__(self,
                      formula=None,
                      inputs=[chainer.link(types.EvaluationLink,  # InheritanceLink
                                           [a,  # PredicateNode A
                                            chainer.link(types.ListLink,  # ListLink
                                                [chainer.link(types.AndLink, [c, b])])])],  # C ANDLink B
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
                      inputs=[self.chainer.link(types.ContextLink,  # ContextLink
                                                [c,
                                                self.chainer.link(types.InheritanceLink,  # InheritanceLink
                                                          [a, b])])],
                      outputs=[self.chainer.link(types.InheritanceLink,  # InheritanceLink
                                            [self.chainer.link(types.AndLink, [c, a]),  # C ANDLink A
                                             self.chainer.link(types.AndLink, [c, b])])])  # C ANDLink B


class ContextToEvaluationRule(Rule):

    def __init__(self, chainer):
        a = chainer.new_variable()
        b = chainer.new_variable()
        c = chainer.new_variable()

        self.chainer = chainer
        Rule.__init__(self,
                      formula=None,
                      inputs=[chainer.link(types.ContextLink,  # ContextLink
                                           [c,  # ConceptNode C
                                            chainer.link(types.EvaluationLink, # EvaluationLink
                                                         [a,  # PredicateNode A
                                                          chainer.link(types.ListLink, [b])])])],
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
        return evaluation_link, tv


class ContextToSubsetRule(Rule):

    def __init__(self, chainer):
        a = chainer.new_variable()
        c = chainer.new_variable()

        self.chainer = chainer
        Rule.__init__(self,
                      formula=contextFormula,
                      inputs=[chainer.link(types.ContextLink, [c, a])],
                      outputs=[chainer.link(types.SubsetLink, [c, a])])


class ContextFreeToSensitiveRule(Rule):

    def __init__(self, chainer):
        a = chainer.new_variable()
        c = chainer.new_variable()

        self.chainer = chainer
        Rule.__init__(self,
                      formula=contextFormula,  # still needs to be implemented
                      inputs=[chainer.link(types.AndLink, [c, a])],
                      outputs=[chainer.link(types.ContextLink, [c, a])])
