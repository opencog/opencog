__author__ = 'sebastian'

from opencog.atomspace import types
from pln.rules import Rule
from pln.rules.formulas import contextFormula

"""
Rules to convert certain types of links into ContextLinks
and to convert these types of links back into ContextLinks.
Rule explanations can be found in
    ../../../learning/RuleEngine/rules/pln/contextualize.scm
    ../../../learning/RuleEngine/rules/pln/decontextualize.scm
    ../../../learning/RuleEngine/rules/pln/context-free-to-contextualize.scm
For further information see also here:
http://wiki.opencog.org/w/ContextLink
"""


class InheritanceToContextRule(Rule):
    """
    InheritanceLink <TV>
        ANDLink
            ConceptNode C
            ConceptNode A
        ANDLink
            ConceptNode C
            ConceptNode B
    |-
    ContextLink <TV>
        ConceptNode C
        InheritanceLink
            ConceptNode A
            ConceptNode B
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
    """
    EvaluationLink <TV>
        PredicateNode A
        ListLink
            ANDLink
                ConceptNode C
                ConceptNode B
    |-
    ContextLink <TV>
        ConceptNode C
        EvaluationLink
            PredicateNode A
                ListLink
                    ConceptNode B
    """
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
    """
    SubsetLink <TV>
        ConceptNode C
        ConceptNode A
    |-
    ContextLink <TV>
        ConceptNode C
        ConceptNode A
    """
    def __init__(self, chainer):
        a = chainer.new_variable()
        c = chainer.new_variable()

        self.chainer = chainer
        Rule.__init__(self,
                      formula=contextFormula,
                      inputs=[chainer.link(types.SubsetLink, [c, a])],
                      outputs=[chainer.link(types.ContextLink, [c, a])])


class ContextToInheritanceRule(Rule):
    """
    ContextLink <TV>
        ConceptNode C
        InheritanceLink
            ConceptNode A
            ConceptNode B
    |-
    InheritanceLink <TV>
        ANDLink
            ConceptNode C
            ConceptNode A
        ANDLink
            ConceptNode C
            ConceptNode B
    """
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
    """
    ContextLink <TV>
        ConceptNode C
        EvaluationLink
            PredicateNode A
            ListLink
                ConceptNode B
    |-
    EvaluationLink <TV>
        PredicateNode A
        ListLink
            ANDLink
                ConceptNode C
                ConceptNode B
    """
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
            print(inputs)
            print(inputs.out[0])
            print("concept_c", concept_c)
            print("concept_b", concept_b)

            and_link = self.chainer.link(types.AndLink, [concept_c, concept_b])
            and_link2 = self.chainer.link(types.AndLink, [concept_b, concept_c])
            print("and_link:", and_link)
            print("and_link2:", and_link2)
            list_link = self.chainer.link(types.ListLink, [and_link])
            evaluation_link = [self.chainer.link(types.EvaluationLink,
                                                [predicate_a, list_link])]
            print(evaluation_link)
        return evaluation_link, [tv]


class ContextToSubsetRule(Rule):
    """
    ContextLink <TV>
        ConceptNode C
        ConceptNode A
    |-
    SubsetLink <TV>
        ConceptNode C
        ConceptNode A
    """
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
    """
    ANDLink
        ConceptNode C<TV1>
        ConceptNode A <TV2>
    |-
    ContextLink <TV3>
        ConceptNode C
        ConceptNode A
    """
    def __init__(self, chainer):
        a = chainer.new_variable()
        c = chainer.new_variable()

        self.chainer = chainer
        Rule.__init__(self,
                      formula=contextFormula,
                      inputs=[chainer.link(types.AndLink, [c, a])],
                      outputs=[chainer.link(types.ContextLink, [c, a])])
