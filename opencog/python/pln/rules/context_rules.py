__author__ = 'sebastian'

from opencog.atomspace import types, TruthValue, get_type_name
from pln.rules import Rule

"""
Rules to convert certain types of links into ContextLinks.
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
                      formula=None,
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
                                            chainer.link(types.ListLink, [  # ListLink
                                                chainer.link(types.AndLink(c, b))])])],  # C ANDLink B
                      outputs=[])

    def custom_compute(self, inputs):
        [inputs] = inputs
        tv = inputs.tv

        if inputs.out[0].type == types.PredicateNode:
            predicate_a = inputs.out[0].type
            concept_c = inputs.out[1].out[0].out[0]
            concept_b = inputs.out[1].out[0].out[1]

            list_link = self.chainer.link(types.ListLink,
                                          [concept_b])
            evaluation_link = self.chainer.link(types.EvaluationLink,
                                                [predicate_a, list_link])
            context_link = [self.chainer.link(types.ContextLink,
                                              [concept_c, evaluation_link])]
        return context_link, [tv]


class SubsetToContext(Rule):

    def __init__(self, chainer):
        a = chainer.new_variable()
        c = chainer.new_variable()

        self.chainer = chainer
        Rule.__init__(self,
                      formula=None,
                      inputs=[chainer.link(types.SubsetLink, [c, a])],
                      outputs=[chainer.link(types.ContextLink, [c, a])])


class ContextToInheritance(Rule):

    def __init__(self, chainer):
        a = chainer.new_variable()
        b = chainer.new_variable()
        c = chainer.new_variable()

        self.chainer = chainer
        Rule.__init__(self,
                      formula=None,
                      inputs=[chainer.link(types.ContextLink,  # ContextLink
                                            [c,
                                             chainer.link(types.InheritanceLink,  # InheritanceLink
                                                          [a, b])])],
                      outputs=[chainer.link(types.InheritanceLink,  # InheritanceLink
                                           [chainer.link(types.AndLink, [c, a]),  # C ANDLink A
                                            chainer.link(types.AndLink, [c, b])])])  # C ANDLink B


class ContextToEvaluation(Rule):




