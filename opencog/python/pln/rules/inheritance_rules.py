from opencog.atomspace import types, TruthValue, get_type_name
import formulas
from pln.rule import Rule

'''
Some Rules evaluate various kinds of logical links based explicitly on
set membership. A set = a ConceptNode. Other Rules calculate them
heuristically, based on set probabilities and logical links.
'''

# Todo: try to separate these rules further into several files by
# category. The rules in this file were under the header 'inheritance
# rules' in rules.py, but may need to be further classified.

__VERBOSE__ = False

BOOLEAN_LINKS = [types.AndLink,
                 types.OrLink,
                 types.NotLink]

FIRST_ORDER_LINKS = [types.InheritanceLink,
                     types.SubsetLink,
                     types.IntensionalInheritanceLink,
                     types.SimilarityLink,
                     types.ExtensionalSimilarityLink,
                     types.IntensionalSimilarityLink]

HIGHER_ORDER_LINKS = [types.ImplicationLink,
                      types.EquivalenceLink]


class InversionRule(Rule):
    """
    A->B entails B->A
    """
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
                      name = "InversionRule<%s>"%(get_type_name(link_type),),
                      outputs=[chainer.link(link_type, [B, A])],
                      inputs=[chainer.link(link_type, [A, B]), A, B],
                      formula=formulas.inversionFormula)


class DeductionRule(Rule):
    """
    A->B, B->C entails A->C
    """
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()
        C = chainer.new_variable()

        Rule.__init__(self,
                      name = "DeductionRule<%s>"%(get_type_name(link_type),),
                      formula=formulas.deductionIndependenceBasedFormula,
                      outputs=[chainer.link(link_type, [A, C])],
                      inputs=[chainer.link(link_type, [A, B]),
                              chainer.link(link_type, [B, C]),
                              B,
                              C])


# Todo: It doesn't have the right formula
class DeductionGeometryRule(Rule):
    """
    A->B, B->C entails A->C. Uses concept geometry.
    """
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()
        C = chainer.new_variable()

        Rule.__init__(self,
            name="DeductionGeometryRule<%s>"%(get_type_name(link_type),),
            formula=formulas.deductionGeometryFormula,
            outputs=[chainer.link(link_type, [A, C])],
            inputs=[chainer.link(link_type, [A, B]),
                    chainer.link(link_type, [B, C])])

# TODO add macro-rules for Abduction and Induction based on Deduction
# and Inversion

'''
deduction
S is M, M is L, then S is L

induction
M is S, M is L, then S is L
invert  same    same

abduction
S is M, L is M, then S is L
        invert
'''


class InductionRule(Rule):
    """
    M->S, M->L, S->L
    """
    def __init__(self, chainer, link_type):
        S = chainer.new_variable()
        M = chainer.new_variable()
        L = chainer.new_variable()

        Rule.__init__(self,
                      name="InductionRule<%s>"%(get_type_name(link_type),),
                      outputs=[chainer.link(link_type, [S, L])],
                      inputs=[chainer.link(link_type, [M, S]),
                              chainer.link(link_type, [M, L]), S, M, L],
                      formula=formulas.inductionFormula)


class AbductionRule(Rule):
    """
    S is M, L is M, S->L
    """
    def __init__(self, chainer, link_type):
        S = chainer.new_variable()
        M = chainer.new_variable()
        L = chainer.new_variable()

        Rule.__init__(self,
                      name="AbductionRule<%s>"%(get_type_name(link_type),),
                      outputs=[chainer.link(link_type, [S, L])],
                      inputs=[chainer.link(link_type, [S, M]),
                              chainer.link(link_type, [L, M]), S, M, L],
                      formula=formulas.abductionFormula)


class TransitiveSimilarityRule(Rule):
    """
    Similarity A B, Similarity B C => Similarity A C
    """
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()
        C = chainer.new_variable()

        Rule.__init__(self,
                      name="TransitiveSimilarityRule<%s>"%(get_type_name(link_type),),
                      formula=formulas.transitiveSimilarityFormula,
                      outputs=[chainer.link(link_type, [A, C])],
                      inputs=[chainer.link(link_type, [A, B]),
                              chainer.link(link_type, [B, C]),
                              A, B, C])


class PreciseModusPonensRule(Rule):
    """
    Given P(A->B) and P(NOT(A)->B) and sA, estimate sB
    """
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()

        notA = chainer.link(types.NotLink, [A])

        Rule.__init__(self,
                      name="PreciseModusPonensRule<%s>"%(get_type_name(link_type),),
                      outputs=[B],
                      inputs=[chainer.link(link_type, [A, B]),
                              chainer.link(link_type, [notA, B]),
                              A],
                      formula=formulas.preciseModusPonensFormula)


class ModusPonensRule(Rule):
    """
    Given P(A->B) and sA, estimate sB
    """
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
                      name="ModusPonensRule<%s>"%(get_type_name(link_type),),
                      outputs=[B],
                      inputs=[chainer.link(link_type, [A, B]),
                              A],
                      formula=formulas.modusPonensFormula)


class SymmetricModusPonensRule(Rule):
    """
    Given (Similarity A B) and sA, estimate sB
    """
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
                      name="SymmetricModusPonensRule<%s>"%(get_type_name(link_type),),
                      outputs=[B],
                      inputs=[chainer.link(link_type, [A, B]),
                              A],
                      formula=formulas.symmetricModusPonensFormula)


class TermProbabilityRule(Rule):
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()

        AB = chainer.link(link_type, [A, B])
        BA = chainer.link(link_type, [B, A])

        Rule.__init__(self,
                      name="TermProbabilityRule<%s>"%(get_type_name(link_type),),
                      outputs=[B],
                      inputs=[AB, BA, A],
                      formula=formulas.termProbabilityFormula)


class InheritanceRule(Rule):
    """
    Create a (mixed) InheritanceLink based on the SubsetLink and
    IntensionalInheritanceLink (based on the definition of mixed
    InheritanceLinks)
    """
    def __init__(self, chainer):
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
                      outputs=[chainer.link(types.InheritanceLink, [A, B])],
                      inputs=[chainer.link(types.SubsetLink, [A, B]),
                              chainer.link(types.IntensionalInheritanceLink,
                                           [A, B])],
                      formula=formulas.inheritanceFormula)


class SimilarityRule(Rule):
    """
    SimilarityLink A B
    |A and B| / |A or B|
    """
    def __init__(self, chainer):
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
                      outputs=[chainer.link(types.SimilarityLink, [A, B])],
                      inputs=[chainer.link(types.AndLink, [A, B]),
                              chainer.link(types.OrLink, [A, B])],
                      formula=formulas.extensionalSimilarityFormula)

class SubsetRule1(Rule):
    """
    SubsetLink A B
    |A and B| / |A|
    = P(B|A)
    """
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
                      name="SubsetRule<%s>"%(get_type_name(link_type),),
                      outputs=[chainer.link(link_type, [A, B])],
                      inputs=[chainer.link(types.AndLink, [A, B]),
                              A],
                      formula=formulas.subsetFormula)

class AndToSubsetRule1(Rule):
    """
    SubsetLink A B
    |A and B| / |A|
    = P(B|A)
    """
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
                      name="AndToSubsetRule1<%s>"%(get_type_name(link_type),),
                      outputs=[chainer.link(link_type, [A, B])],
                      inputs=[chainer.link(types.AndLink, [A, B]),
                              A],
                      formula=formulas.subsetFormula)

class AndToSubsetRuleN(Rule):
    """
    SubsetLink And(A B C) D
    |And(A B C D)| / |And A B C|
    = P(B|A)
    """
    def __init__(self, chainer, link_type, N):
        vars = chainer.make_n_variables(N)
        
        lhs = chainer.link(types.AndLink, vars[:-1])
        rhs = vars[-1]

        Rule.__init__(self,
                      name="AndToSubsetRuleN<%s,%s>"%(get_type_name(link_type),N),
                      outputs=[chainer.link(link_type, [lhs, rhs])],
                      inputs=[chainer.link(types.AndLink, vars),
                              lhs],
                      formula=formulas.subsetFormula)

