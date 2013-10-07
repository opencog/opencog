from opencog.atomspace import types, TruthValue

import pln.formulas as formulas
import pln.temporalFormulas

import math

class Rule(object):

    def __init__ (self, outputs, inputs, formula, multi_inputs=None):
        '''@outputs is one or more Trees representing the structure of Atom (usually a Link)
    to be produced by this Rule. If it's a variable then any kind of Atom
    can be produced.
    @inputs (list of Trees) specifies what kinds of Atoms
    are necessary to produce it. There should be variables used in the head
    that also appear in the goals. This means they will be the same atom.
    It's OK to reuse the same variable numbers in different Rules - they'll
    be converted to unique variables automatically.
    You can use either @formula or @tv. You specify a formula method from formulas.py;
    it will be called with the TVs of the relevant Atoms, to calculate the TruthValue for
    the resulting Atom.'''
        assert type(outputs) == list
        assert type(inputs) == list

        self._outputs = outputs
        self._inputs = inputs
        self._multi_inputs = multi_inputs

        self.formula = formula
        self.name = self.__class__.__name__
    
    def calculate(self, input_atoms):
        '''Compute the output TV(s) based on the input atoms'''
        tvs = [atom.tv for atom in input_atoms]
        return self.formula(tvs)

    def standardize_apart_input_output(self, chainer):
        new_inputs = []
        new_outputs = []
        dic = {}

        for template in self._inputs:
            new_template = chainer.standardize_apart(template, dic)
            new_inputs.append(new_template)

        for template in self._outputs:
            new_template = chainer.standardize_apart(template, dic)
            new_outputs.append(new_template)

        return (new_inputs, new_outputs)

# Inheritance Rules

class InversionRule(Rule):
    '''A->B entails B->A'''
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
            outputs= [chainer.link(link_type, [B, A])],
            inputs=  [chainer.link(link_type, [A, B]),
                      A, B],
            formula= formulas.inversionFormula)

class DeductionRule(Rule):
    '''A->B, B->C entails A->C'''
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()
        C = chainer.new_variable()

        Rule.__init__(self,
            formula= formulas.deductionSimpleFormula,
            outputs= [chainer.link(link_type, [A, C])],
            inputs=  [chainer.link(link_type, [A, B]),
                      chainer.link(link_type, [B, C]),
                      A, B, C])

# TODO add macro-rules for Abduction and Induction based on Deduction and Inversion
class InductionRule(Rule):
    '''A->B, A->C entails B->C'''
    def __init__(self, chainer, link_type):
        A = chainer.new_variable()
        B = chainer.new_variable()
        C = chainer.new_variable()

        Rule.__init__(self,
            outputs= [chainer.link(link_type, [B, C])],
            inputs=  [chainer.link(link_type, [A, B]),
                      chainer.link(link_type, [A, C])],
            formula= None)
            
    def compute(inputs):
        pass

# TODO MemberToInheritanceRule

# And/Or/Not Rules

class NotCreationRule(Rule):
    '''A => NotLink(A)'''
    def __init__(self, chainer):
        A = chainer.new_variable()

        Rule.__init__(self,
            formula= formulas.notFormula,
            outputs= [chainer.link(types.NotLink, [A])],
            inputs= [A])

class NotEliminationRule(Rule):
    '''NotLink(A) => A'''
    def __init__(self, chainer):
        A = chainer.new_variable()

        Rule.__init__(self,
            formula= formulas.notFormula,
            outputs= [A],
            inputs=  [chainer.link(types.NotLink, [A])])

def make_n_variables(chainer, N):
    return [chainer.new_variable() for i in xrange(0, N)]

# TODO These should take account of dependencies in some cases

def create_and_or_rules(chainer, min_n, max_n):
    rules = []
    for n in min_n, max_n:
        rules.append(AndCreationRule(chainer, n))
        rules.append(OrCreationRule(chainer, n))
        rules.append(AndEliminationRule(chainer, n))
        rules.append(OrEliminationRule(chainer, n))

    return rules

class AndCreationRule(Rule):
    '''Take a set of N atoms and create AndLink(atoms)'''
    def __init__(self, chainer, N):
        atoms = make_n_variables(chainer, N)

        Rule.__init__(self,
            formula= formulas.andSymmetricFormula,
            outputs= [chainer.link(types.AndLink, atoms)],
            inputs=  atoms)

class OrCreationRule(Rule):
    '''[A, B...] => Or(A, B...)'''
    def __init__(self, chainer, N):
        atoms = make_n_variables(chainer, N)

        Rule.__init__(self,
            formula= formulas.orFormula,
            outputs= [chainer.link(types.OrLink, atoms)],
            inputs=  atoms)

class IntersectionRule(Rule):
    '''Compute the membership of AndLink(concepts) based on set theory. And(A B) is defined as
       the set of things that are members of both A and B (set intersection).
       Inputs: MemberLink(x, A) or MemberLink(x, B) for many objects/etc.'''
    def __init__(self, chainer, N):
        x = chainer.new_variable()
        variables = make_n_variables(chainer, N)

        inputs = [chainer.link(types.MemberLink, [x, var]) for var in variables]
        and_link = chainer.link(types.AndLink, variables)

        Rule.__init__(self,
            formula= None,
            outputs= [chainer.link(types.MemberLink, [x, and_link])],
            inputs=  inputs)

class SubsetEvaluationRule(Rule):
    '''Compute Subset(A B) which is equivalent to P(x in B| x in A).'''
    def __init__(self, chainer):
        x = chainer.new_variable()
        A = chainer.new_variable()
        B = chainer.new_variable()

        inputs= [chainer.link(types.MemberLink, [x, A]),
                 chainer.link(types.MemberLink, [x, B])]

        Rule.__init__(self,
            formula= formulas.subsetEvaluationFormula,
            outputs= [chainer.link(types.SubsetLink, [A, B])],
            inputs=  inputs)

# Elimination Rules

class AbstractEliminationRule(Rule):
    def __init__(self, chainer, N, link_type):
        atoms = make_n_variables(chainer, N)

        Rule.__init__(self,
            formula= None,
            outputs= atoms,
            inputs=  [chainer.link(link_type, atoms)])

class AndEliminationRule(AbstractEliminationRule):
    '''AndLink(atoms) => atoms'''
    def __init__(self, chainer, N):
        AbstractEliminationRule.__init__(self, chainer, N, link_type= types.AndLink)

    def compute(self, atoms):
        [and_atom] = atoms
        outputs = and_atom.out
        N = len(outputs)

        # assume independence, i.e. P(A^B^C...) = P(A)P(B)P(C)...
        # therefore P(A) = Nth root of P(AndLink)
        # same for P(B) etc
        individual_tv = math.pow(and_atom.tv, 1.0/N)

        output_tvs = [individual_tv for out in outputs]

        return output_tvs

class OrEliminationRule(AbstractEliminationRule):
    '''Take OrLink(atoms) and produce all of the atoms separately'''
    def __init__(self, chainer, N):
        AbstractEliminationRule.__init__(self, chainer, N, link_type= types.OrLink)

    def compute(self, atoms):
        [or_atom] = atoms
        outputs = or_atom.out
        N = len(outputs)

        # TODO this formula is wrong: it assumes P(A or B or C...) = P(A)+P(B)+P(C)...
        # therefore P(A) = P(OrLink)/N
        # same for P(B) etc
        individual_mean = or_atom.tv.mean/N
        count = 1 # hack

        output_tvs = [TruthValue(individual_mean, count) for out in outputs]

        return output_tvs

class NotEliminationRule(AbstractEliminationRule):
    '''NotLink(NotLink(A)) => A'''
    pass

class EvaluationToMemberRule(Rule):
    '''Turns EvaluationLink(PredicateNode P, argument) into 
       MemberLink(argument, ConceptNode "SatisfyingSet(P)".
       The argument can either be a single Node/Link or a ListLink or arguments.'''
    def __init__(self, chainer):
        P = chainer.new_variable()
        ARG = chainer.new_variable()

        self.chainer = chainer
        Rule.__init__(self,
                      formula= None,
                      inputs=  [chainer.link(types.EvaluationLink, [P, ARG])],
                      outputs= [])

    def custom_compute(self, inputs):
        [eval_link] = inputs
        [predicate, arg] = eval_link.out

        concept_name = 'SatisfyingSet(%s)' % (predicate.name,)
        set_node = self.chainer.node(types.ConceptNode, concept_name)

        member_link = self.chainer.link(types.MemberLink, [arg, set_node])
        tv = eval_link.tv

        return ([member_link], [tv])

class MemberToInheritanceRule(Rule):
    '''MemberLink(A B) => InheritanceLink(A B)'''
    def __init__(self, chainer):
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
            formula= formulas.mem2InhFormula,
            outputs= [chainer.link(types.InheritanceLink, [A, B])],
            inputs=  [chainer.link(types.MemberLink, [A, B])])

class AttractionEvaluationRule(Rule):
    '''Creates ExtensionalAttractionLink(A, B) <s>.
       s = Subset(A, B).s - Subset(Not(A), B).
       Attraction founder rich = % of founders who are rich - % normal people who are rich'''
    def __init__(self, chainer):
        self._chainer = chainer
        A = chainer.new_variable()
        B = chainer.new_variable()

        Rule.__init__(self,
            formula= None,
            outputs= [chainer.link(types.AttractionLink, [A, B])],
            inputs= None)

class ASSOCEvaluationRule(Rule):
    '''Creates the extensional association set of ConceptNode C (called ASSOC_ext(C).
    MemberLink(e, ASSOC(C)).tv = Func(Subset(e, C), Subset(Not(e), C)).'''
    pass

